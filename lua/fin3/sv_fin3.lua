local deg, rad, asin, cos, abs, sign, pi = math.deg, math.rad, math.asin, math.cos, math.abs, Fin3.sign, math.pi
local min, max, remap, clamp, sqrt = math.min, math.max, math.Remap, math.Clamp, math.sqrt
local localToWorldVector = Fin3.localToWorldVector
local roundVectorToAxis = Fin3.roundVectorToAxis
local applyForceOffsetFixed = Fin3.applyForceOffsetFixed
local getRootParent = Fin3.getRootParent
local dt = engine.TickInterval()

-- Upper bound of d(normal force coef)/d(AoA) per radian over all models (flat plate pre-stall is the steepest, ~7.2)
local maxNormalCoefSlope = 7.2

-- Max share of the opposing airflow all fins on one root may cancel in a single tick. Explicit forces go unstable
-- around 1 (with the low-pass filter and a tick of lag), at 0.5 any oscillation still dies out within ~20 ticks
local maxRootStiffness = 0.5

--- Velocity change (in/s) along dir at pos, per unit impulse (kg*in/s) applied there along dir - i.e. 1 / effective mass
local function pointInvMass(phys, pos, dir)
    local invInertia = phys:GetInvInertia() -- 1 / kg*m^2, local principal axes, 0 when frozen
    local r = (phys:WorldToLocal(pos) - phys:GetMassCenter()) / 39.3701
    local rxd = r:Cross(phys:WorldToLocalVector(dir))

    return phys:GetInvMass() + invInertia.x * rxd.x ^ 2 + invInertia.y * rxd.y ^ 2 + invInertia.z * rxd.z ^ 2
end

Fin3.fin = {} -- Fin class

--[[
Fin creation data table structure:
    finType: string [model name]
    upAxis: vector
    forwardAxis: vector
    inducedDrag: number [0 - 1]
    camber: number [0 - 100]
    efficiency: number [0.1 - 1.5]
    disableLowPass: boolean
--]]

--- Adds a new fin to an entity
---@param ply Player Player that created the fin
---@param ent Entity Entity to add the fin to
---@param data table Fin creation data table
function Fin3.fin:new(ply, ent, data)
    if not IsValid(ent) or not Fin3.allowedClasses[ent:GetClass()] or not IsValid(ent:GetPhysicsObject()) then return end
    if not data or not isvector(data.upAxis) or not isvector(data.forwardAxis) then return end

    local rootPhys = getRootParent(ent):GetPhysicsObject()
    if not IsValid(rootPhys) then return end

    local existingFin = Fin3.fins[ent]

    if not existingFin and IsValid(ply) then
        local finLimit = GetConVar("sbox_max_fin3"):GetInt()
        local currentFinCount = Fin3.playerFinCount[ply] or 0

        if currentFinCount >= finLimit then
            ply:LimitHit("Fin3")

            return
        end

        Fin3.playerFinCount[ply] = currentFinCount + 1
    end

    local fin = {}
    setmetatable(fin, {__index = Fin3.fin})

    fin.ent = ent
    fin.owner = existingFin and existingFin.owner or ply -- Keep counting the fin against whoever created it
    fin.massCenter = ent:GetPhysicsObject():GetMassCenter()
    fin.upAxis = data.upAxis
    fin.forwardAxis = data.forwardAxis
    fin.rightAxis = data.forwardAxis:Cross(data.upAxis)
    fin.selfPhys = ent:GetPhysicsObject()

    if data.finType == "cambered" or data.finType == "symmetrical" then
        fin.finType = "standard"
    else
        fin.finType = data.finType
    end

    -- Unknown or hidden types (e.g. a client setting fin3_fintype to anything) would error in the Tick hook every tick
    local model = Fin3.models[fin.finType]
    if not model or model.hidden then
        fin.finType = "standard"
    end

    if data.zeroLiftAngle then
        fin.camber = data.zeroLiftAngle / 8 * 100
    else
        fin.camber = data.camber or 0
    end
    fin.camber = clamp(fin.camber, 0, 100)

    fin.efficiency = clamp(tonumber(data.efficiency or data.forceMultiplier) or 1, 0.1, 1.5) -- Account for old versions

    if data.inducedDrag == nil then
        fin.inducedDrag = 1
    else
        if isbool(data.inducedDrag) then
            fin.inducedDrag = data.inducedDrag and 1 or 0
        else
            fin.inducedDrag = clamp(data.inducedDrag, 0, 1)
        end
    end

    if data.lowpass ~= nil then
        fin.disableLowPass = not data.lowpass
    else
        fin.disableLowPass = data.disableLowPass or false
    end

    fin:calcSurfaceArea()

    fin.velVector = vector_origin
    fin.velNorm = vector_origin
    fin.velMsSqr = 0
    fin.fwdVelRatio = 0

    fin.liftVector = vector_origin
    fin.liftForceNewtons = 0
    fin.lastLiftForceNewtons = 0
    fin.liftInducedDragCoef = 0
    fin.lastDragForceNewtons = 0
    fin.dragForceNewtons = 0

    fin.lastPos = ent:LocalToWorld(fin.massCenter)

    fin.angleOfAttack = 0

    rootPhys:EnableDrag(false)
    local _, rotDamping = rootPhys:GetDamping()
    rootPhys:SetDamping(0, rotDamping)

    ent:SetNW2String("fin3_finType", fin.finType)
    ent:SetNW2Vector("fin3_upAxis", fin.upAxis)
    ent:SetNW2Vector("fin3_forwardAxis", fin.forwardAxis)
    ent:SetNW2Vector("fin3_rightAxis", fin.rightAxis)
    ent:SetNW2Float("fin3_camber", fin.camber)
    ent:SetNW2Float("fin3_efficiency", fin.efficiency)
    ent:SetNW2Float("fin3_inducedDrag", fin.inducedDrag)
    ent:SetNW2Float("fin3_aoa", 0)
    ent:SetNW2Bool("fin3_disableLowPass", fin.disableLowPass)

    fin.rootPhys = rootPhys

    Fin3.fins[ent] = fin

    if IsValid(ent.Fin2_Ent) then
        ent.Fin2_Ent:Remove()
        ent:SetNWFloat("efficency", 0)
    end

    Fin3.transmitFin(ent)

    duplicator.StoreEntityModifier(ent, "fin3", data)
end

duplicator.RegisterEntityModifier("fin3", function(...)
    Fin3.fin:new(...)
end)

function Fin3.fin:calcSurfaceArea()
    local ent = self.ent

    if ent:GetClass() == "primitive_airfoil" then
        local controlSurfaceMode = ent:GetPrimCSOPT()

        local sweepAngle = -ent:GetPrimSWEEP()

        local effectiveSpan = ent:GetPrimSPAN() / 39.3701 / cos(rad(sweepAngle))
        local rootChord = ent:GetPrimCHORDR() / 39.3701
        local tipChord = ent:GetPrimCHORDT() / 39.3701
        local finSurfaceArea = (rootChord + tipChord) / 2 * effectiveSpan

        if controlSurfaceMode > 0 and controlSurfaceMode ~= 2 then
            local xLength = ent:GetPrimCSXLEN()
            local yLength = ent:GetPrimCSYLEN()
            local yPos = ent:GetPrimCSYPOS()

            local controlSurfaceSpan = effectiveSpan * min(1 - yPos, yLength)

            -- Chord at the tip of the control surface is the chord it uses along its whole length
            local controlSurfaceChord = remap(min(yPos + yLength, 1), 0, 1, rootChord, tipChord) * xLength

            local controlSurfaceArea = controlSurfaceChord * controlSurfaceSpan

            if controlSurfaceMode == 3 then -- Inverse clipping enabled, whole primitive is the control surface
                self.aspectRatio = controlSurfaceSpan^2 / controlSurfaceArea
                self.surfaceArea = controlSurfaceArea
            else -- Control surface is enabled and is clipped out of the main primitive
                self.surfaceArea = finSurfaceArea - controlSurfaceArea
                self.aspectRatio = effectiveSpan^2 / finSurfaceArea -- Aspect ratio of the whole fin, disregarding the control surface cut
            end
        else
            self.surfaceArea = finSurfaceArea
            self.aspectRatio = effectiveSpan^2 / self.surfaceArea
        end

        ent:SetNW2Float("fin3_sweepAngle", sweepAngle)
    else
        local obbSize = ent:OBBMaxs() - ent:OBBMins()

        local span = abs(obbSize:Dot(roundVectorToAxis(self.rightAxis)))
        local chord = abs(obbSize:Dot(roundVectorToAxis(self.forwardAxis)))

        self.surfaceArea = span * chord * 0.00064516 -- in^2 to m^2
        self.aspectRatio = span / chord
    end
    self.invAspectRatio = 1 / self.aspectRatio

    ent:SetNW2Float("fin3_surfaceArea", self.surfaceArea)
    ent:SetNW2Float("fin3_aspectRatio", self.aspectRatio)
end

function Fin3.fin:getVelocity()
    local curPos = self.ent:LocalToWorld(self.massCenter)
    local vel = (curPos - self.lastPos) / dt

    self.lastPos = curPos

    return vel
end

--- Calculates velocity, angle of attack, and lift vector
function Fin3.fin:calcBaseData()
    local ent = self.ent

    if not IsValid(self.selfPhys) or self.selfPhys ~= ent:GetPhysicsObject() then
        local newPhys = ent:GetPhysicsObject()
        self.selfPhys = newPhys

        if not IsValid(newPhys) then return end

        self.massCenter = newPhys:GetMassCenter()
        self:calcSurfaceArea()
        newPhys:EnableDrag(false)
        local _, rotDamping = newPhys:GetDamping()
        newPhys:SetDamping(0, rotDamping)
    end

    local newRootPhys = getRootParent(self.ent):GetPhysicsObject()

    if not IsValid(newRootPhys) then
        self.rootPhys = nil
        self.lastPos = ent:LocalToWorld(self.massCenter)
        return
    end

    if not IsValid(self.rootPhys) or newRootPhys ~= self.rootPhys then
        newRootPhys:EnableDrag(false)
        local _, rotDamping = newRootPhys:GetDamping()
        newRootPhys:SetDamping(0, rotDamping)

        self.rootPhys = newRootPhys
    end

    self.velVector = self:getVelocity()

    if self.velVector == vector_origin then return end

    self.forwardVel = self.velVector:Dot(localToWorldVector(ent, self.forwardAxis))
    self.rightVel = self.velVector:Dot(localToWorldVector(ent, self.rightAxis))

    local forwardAndRightVel = abs(self.forwardVel) + abs(self.rightVel)
    self.fwdVelRatio = self.forwardVel / forwardAndRightVel

    self.velNorm = self.velVector:GetNormalized()
    self.velMsSqr = (self.velVector:Length() / 39.3701) ^ 2

    local worldUpAxis = localToWorldVector(ent, self.upAxis):GetNormalized()
    local upVelNorm = self.velNorm:Dot(worldUpAxis)

    if abs(upVelNorm) > 0.999 then
        self.angleOfAttack = -90 * sign(upVelNorm) -- Same sign as the asin branch below
    else
        self.angleOfAttack = -deg(asin(self.velNorm:Dot(worldUpAxis)))
    end

    ent:SetNW2Float("fin3_aoa", self.angleOfAttack)

    local side = self.velNorm:Cross(worldUpAxis)
    self.liftVector = -self.velNorm:Cross(side)
end

function Fin3.fin:calcLiftForceNewtons()
    local flatModel = Fin3.models.flat
    local curModel = Fin3.models[self.finType]
    local liftCoef = 0
    local AoA = self.angleOfAttack
    local liftCoefFlat = Fin3.calcLiftCoef(abs(AoA), flatModel.stallAngle, flatModel.liftCoefPeakPreStall, flatModel.liftCoefPeakPostStall)
    liftCoefFlat = liftCoefFlat * sign(AoA)
    local fwdVelRatio = 0

    if self.forwardVel > 0 and self.finType ~= "flat" then
        fwdVelRatio = self.fwdVelRatio

        local AoAFinal = AoA
        local stallAngleMod = 0
        local negativeAoACoefPenalty = 0

        if curModel.canCamber then
            local camber = self.camber
            local zeroLiftAngle = camber * 0.08
            AoAFinal = (AoA + zeroLiftAngle + 90) % 180 - 90

            if not curModel.customLiftCoef then
                if AoAFinal < 0 then
                    negativeAoACoefPenalty = camber / 100 * curModel.negativeAoACamberPeakLiftCoefPenalty
                    stallAngleMod = -camber / 100 * curModel.negativeAoACamberPeakStallAnglePenalty
                else
                    stallAngleMod = camber / 100 * curModel.positiveAoACamberStallAngleBonus
                end
            end
        end

        local liftCoefForward

        if curModel.customLiftCoef then
            liftCoefForward = curModel.customLiftCoef(abs(AoAFinal))
        else
            liftCoefForward = Fin3.calcLiftCoef(abs(AoAFinal), curModel.stallAngle + stallAngleMod, curModel.liftCoefPeakPreStall - negativeAoACoefPenalty, curModel.liftCoefPeakPostStall)
        end

        liftCoefForward = liftCoefForward * sign(AoAFinal)

        liftCoef = Lerp(fwdVelRatio, liftCoefFlat, liftCoefForward)
    else
        liftCoef = liftCoefFlat
    end

    -- Cdi = (Cl^2) / (pi * AR * e)
    local inducedDrag = self.inducedDrag
    if inducedDrag > 0 then
        local aspectRatio = Lerp(fwdVelRatio, self.invAspectRatio, self.aspectRatio)
        self.liftInducedDragCoef = (liftCoef ^ 2) / (pi * aspectRatio * Fin3.finEfficiency) * inducedDrag
    end

    local lift = 0.5 * liftCoef * Fin3.airDensity * self.surfaceArea * self.velMsSqr * self.efficiency

    -- low pass filter
    -- self.liftForceNewtons = self.lastLiftForceNewtons * (1 - alpha) + lift * alpha
    self.liftForceNewtons = self.disableLowPass and lift or (self.lastLiftForceNewtons * 0.6 + lift * 0.4)
    self.lastLiftForceNewtons = self.liftForceNewtons
end

function Fin3.fin:calcDragForceNewtons()
    local flatModel = Fin3.models.flat
    local curModel = Fin3.models[self.finType]
    local dragCoef = 0
    local AoA = self.angleOfAttack
    local dragCoefFlat = Fin3.calcDragCoef(abs(AoA), flatModel.stallAngle, flatModel.dragCoefPeakPreStall, flatModel.dragCoefPeakPostStall)

    if self.forwardVel > 0 and self.finType ~= "flat" then
        local fwdVelRatio = self.fwdVelRatio

        local zeroLiftAngle = self.camber * 0.08
        local AoAFinal = (AoA + zeroLiftAngle + 90) % 180 - 90

        local dragCoefForward

        if curModel.customDragCoef then
            dragCoefForward = curModel.customDragCoef(abs(AoAFinal))
        else
            dragCoefForward = Fin3.calcDragCoef(abs(AoAFinal), curModel.stallAngle, curModel.dragCoefPeakPreStall, curModel.dragCoefPeakPostStall)
        end

        dragCoef = abs((dragCoefForward * fwdVelRatio) + (dragCoefFlat * (1 - fwdVelRatio)))
    else
        dragCoef = dragCoefFlat
    end

    dragCoef = dragCoef + self.liftInducedDragCoef
    self.dragCoef = dragCoef

    local drag = 0.5 * dragCoef * Fin3.airDensity * self.surfaceArea * self.velMsSqr * self.efficiency

    self.dragForceNewtons = self.disableLowPass and drag or (self.lastDragForceNewtons * 0.6 + drag * 0.4)
    self.lastDragForceNewtons = self.dragForceNewtons
end

function Fin3.fin:applyForce(force)
    applyForceOffsetFixed(self.rootPhys, force, self.ent:LocalToWorld(self.massCenter))
end

--- Aerodynamic forces are applied explicitly once per tick. Once a single tick's force is strong enough to reverse
--- the airflow it reacts to (high speed, big/efficient fin, light root, long lever arm), the vehicle starts shaking
--- and eventually explodes. The Tick hook sums g over all fins on a root and, only when it exceeds maxRootStiffness,
--- scales their forces down to that limit - normal flight is left untouched.
---@return number g How much of the opposing airflow one tick of this fin's force would cancel (dimensionless)
function Fin3.fin:calcStiffness()
    local phys = self.rootPhys
    local pos = self.ent:LocalToWorld(self.massCenter)
    local qA = 0.5 * Fin3.airDensity * self.surfaceArea * self.efficiency * sqrt(self.velMsSqr) -- kg/s

    -- d(force)/d(velocity) along the fin normal (lift and normal force) and along the airflow (drag)
    local normalDamping = qA * maxNormalCoefSlope
    local dragDamping = 2 * qA * (self.dragCoef or 0)

    local worldUpAxis = localToWorldVector(self.ent, self.upAxis):GetNormalized()

    return dt * max(
        normalDamping * pointInvMass(phys, pos, worldUpAxis),
        dragDamping * pointInvMass(phys, pos, self.velNorm)
    )
end

--- Calculates this tick's lift and drag, sets self.stiffness if there is a force to apply
function Fin3.fin:think()
    self.stiffness = nil

    self:calcBaseData()

    if not IsValid(self.rootPhys) then return end

    if self.velVector == vector_origin then
        self.ent:SetNW2Vector("fin3_liftVector", vector_origin)
        self.ent:SetNW2Vector("fin3_dragVector", vector_origin)

        self.lastPos = self.ent:LocalToWorld(self.massCenter)
        self.lastLiftForceNewtons = 0
        self.lastDragForceNewtons = 0

        return
    end

    self:calcLiftForceNewtons()
    self:calcDragForceNewtons()

    self.stiffness = self:calcStiffness()
end

---@param stability number Multiplier from the Tick hook, see Fin3.fin:calcStiffness
function Fin3.fin:applyForces(stability)
    local finalLiftVector = self.liftVector * self.liftForceNewtons * stability
    self.ent:SetNW2Vector("fin3_liftVector", finalLiftVector)

    local finalDragVector = -self.velNorm * self.dragForceNewtons * stability
    self.ent:SetNW2Vector("fin3_dragVector", finalDragVector)

    local totalForce = finalLiftVector + finalDragVector

    self:applyForce(totalForce * 39.3701 * dt)
end

function Fin3.fin:remove()
    local ent = self.ent

    if IsValid(ent) then
        ent:SetNW2String("fin3_finType", nil)
        ent:SetNW2Vector("fin3_upAxis", nil)
        ent:SetNW2Vector("fin3_forwardAxis", nil)
        ent:SetNW2Vector("fin3_rightAxis", nil)
        ent:SetNW2Float("fin3_efficiency", nil)
        ent:SetNW2Float("fin3_surfaceArea", nil)
        ent:SetNW2Float("fin3_aspectRatio", nil)
        ent:SetNW2Float("fin3_camber", nil)
        ent:SetNW2Float("fin3_inducedDrag", nil)
        ent:SetNW2Float("fin3_aoa", nil)
        ent:SetNW2Bool("fin3_disableLowPass", nil)
        ent:SetNW2Vector("fin3_liftVector", nil)
        ent:SetNW2Vector("fin3_dragVector", nil)
        ent:SetNW2Float("fin3_sweepAngle", nil)
    end

    Fin3.fins[ent] = nil

    -- Only give Source's drag back to the vehicle once no other fin is still flying it
    local rootPhys = self.rootPhys
    if IsValid(rootPhys) then
        local rootShared = false

        for _, otherFin in pairs(Fin3.fins) do
            if otherFin.rootPhys == rootPhys then
                rootShared = true
                break
            end
        end

        if not rootShared then
            rootPhys:EnableDrag(true)
        end
    end

    local owner = self.owner
    if IsValid(owner) and Fin3.playerFinCount[owner] then
        Fin3.playerFinCount[owner] = Fin3.playerFinCount[owner] - 1
    end

    duplicator.ClearEntityModifier(self.ent, "fin3")
end

hook.Add("Tick", "fin3_think", function()
    local rootStiffness = {}

    for _, fin in pairs(Fin3.fins) do
        if not IsValid(fin.ent) then
            fin:remove()
        else
            fin:think()

            if fin.stiffness then
                rootStiffness[fin.rootPhys] = (rootStiffness[fin.rootPhys] or 0) + fin.stiffness
            end
        end
    end

    -- Fins on the same root push the same body, so they have to be stabilized together
    for _, fin in pairs(Fin3.fins) do
        if fin.stiffness then
            local stiffness = rootStiffness[fin.rootPhys]
            fin:applyForces(stiffness > maxRootStiffness and maxRootStiffness / stiffness or 1)
        end
    end
end)

net.Receive("fin3_networkfinids", function(_, ply)
    Fin3.transmitAllFins(ply)
end)
