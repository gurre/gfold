package gfold

// Nonconvex Minimum Landing Error Problem
func minimumLandingError() {
	// min t f ,Tc Er(t f ) − q (3)

	// s.t. x˙(t)= A(ω)x(t)+B (g + Tc(t) m)  ⎫
	//                                       ⎬ ∀t ∈ [0,t f ] (4)
	// m˙(t) = −αTc(t)                       ⎭

	// x(t) ∈ X ∀ t ∈ [0,t f ] (5)

	// 0 < ρ1 ≤ Tc(t) ≤ ρ2, nˆT Tc(t) ≥ Tc(t) cos θ (6)

	//  Equation (7) defines the initial mass of the lander and ensures that no more fuel than available is used.
	// m(0) = m0, m(t f ) ≥ m0 − m f > 0 (7)

	// Equation (8) defines the initial position and velocity of the lander.
	// r(0) = r0, r˙(0) = ˙r0 (8)

	// (9) constrains the final altitude and the velocity. The time of flight t f is an optimization variable and is not fixed a priori.
	// eT 1 r(t f ) = 0, r˙(t f ) = 0. (9)
}

// Nonconvex Minimum Fuel Problem
func minimumFuel() {
	// max t f ,Tc m(t f ) − m(0) = min t f ,Tc(·) t f 0 αTc(t) dt s.t. (10)

	// dynamics and constraints given by (4)–(9)

	// Er(t f ) − q≤d∗ P1 − q. (11)

	// The glide slope constraint (12) ensures that the trajectory to the target cannot be too shallow or go subsurface.
	// X = (r, r˙) ∈ R6 : ˙r ≤ Vmax, E r − r(t f ) −cT r − r(t f ) ≤ 0 (12)

	// c  e1 tan γgs, γgs ∈ (0,π/2). (13)
}

func (g *Gfold) SoftLandTrajectory() (
	thrustAngle float64,
) {
	minimumLandingError()

	if g.optMinMinimumFuelUse {
		minimumFuel()
	}

	return
}
