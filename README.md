# G-FOLD in Golang

Guidance for Fuel Optimal Landing Diverts (G-FOLD) implementation in Golang based on the works of Ackimese, Carson, and Blackmore in 2013. Popularized by SpaceX and Falcon 9.

The soft landing is the final phase of a planetary EDL (entry, descent, and landing), and it is also referred to as the powered descent landing stage. The G-FOLD algoritm optimizes for two problems:

- Landing as close as possible to a target (Nonconvex Minimum Landing Error Problem).
- Consuming the least fuel possible (Nonconvex Minimum Fuel Problem).

## Installation

```bash
go get github.com/gurre/gfold
```

## Usage

```go
// Init library with options
g := gfold.New(
    gfold.WithMinimalFuelUse()
)

// Falcon 9
var (
    dryMass          = 22200 // kg
    fuelMass         = 13400 // kg
    minThrottleLimit = 0.4 // percent
    maxThrottleLimit = 1.0 // percent
    maxThrust        = 845000 * 3 // N
    maxThrustAngle   = 120 // rad
    isp              = 282.0 // s
    maxVelocity      = 1300 // m/s
    gravityAccel     = 9.80665 // m/s^2
    maxG             = 3 //
    glideslope       = 1 // rad

    // Lander position
    xPos = 2400 // m
    yPos = 450 // m
    zPos = -330 // m

    // Lander Velocity
    xVel = -10 // m/s
    yVel = -40 // m/s
    zVel = 10 // m/s

    // thrust vector reference direction
    thrustVectorX = 1
    thrustVectorY = 0
    thrustVectorZ = 0

    // planetary angular velocity
    bodyωX = 0.0000291 // m/s
    bodyωY = 0 // m/s
    bodyωZ = 0.0000668 // m/s

    // final position target
    targetXPos = 0 // m
    targetYPos = 0 // m
    targetZPos = 0 // m

    // final velocity target
    targetXVel = 0 // m/s
    targetYVel = 0 // m/s
    targetZVel = 0 // m/s
)

// Calculate trajectory
g.SoftLandTrajectory()

// pass on result to control in order to actuate

```

## References

- [Flight testing of trajectories computed by g-fold:
  fuel optimal large divert guidance algorithm for
  planetary landing](https://www.researchgate.net/publication/258676350_G-FOLD_A_Real-Time_Implementable_Fuel_Optimal_Large_Divert_Guidance_Algorithm_for_Planetary_Pinpoint_Landing)
- https://github.com/jonnyhyman/G-FOLD-Python
