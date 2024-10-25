package gfold

import (
	"math"
)

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

	// Define constants and initial conditions
	const (
		rho1 = 0.1
		rho2 = 1.0
		alpha = 0.1
		m0 = 1000.0
		mf = 100.0
		tf = 100.0
	)

	var (
		x = [6]float64{0, 0, 0, 0, 0, 0}
		Tc = [3]float64{0, 0, 0}
		m = m0
	)

	// Define the dynamics and constraints
	A := func(omega [3]float64) [6][6]float64 {
		return [6][6]float64{
			{0, 0, 0, 1, 0, 0},
			{0, 0, 0, 0, 1, 0},
			{0, 0, 0, 0, 0, 1},
			{0, 0, 0, 0, omega[2], -omega[1]},
			{0, 0, 0, -omega[2], 0, omega[0]},
			{0, 0, 0, omega[1], -omega[0], 0},
		}
	}

	B := [6][3]float64{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}

	g := [3]float64{0, 0, -9.81}

	// Define the optimization problem
	for t := 0.0; t <= tf; t += 0.1 {
		// Update the state
		xDot := [6]float64{}
		for i := 0; i < 6; i++ {
			for j := 0; j < 6; j++ {
				xDot[i] += A([3]float64{0, 0, 0})[i][j] * x[j]
			}
			for j := 0; j < 3; j++ {
				xDot[i] += B[i][j] * (g[j] + Tc[j]/m)
			}
		}

		for i := 0; i < 6; i++ {
			x[i] += xDot[i] * 0.1
		}

		// Update the mass
		mDot := -alpha * math.Sqrt(Tc[0]*Tc[0]+Tc[1]*Tc[1]+Tc[2]*Tc[2])
		m += mDot * 0.1

		// Check constraints
		if m < m0-mf {
			break
		}
		if x[2] < 0 {
			break
		}
		if math.Sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]) > 0 {
			break
		}
	}
}

// Nonconvex Minimum Fuel Problem
func minimumFuel() {
	// max t f ,Tc m(t f ) − m(0) = min t f ,Tc(·) t f 0 αTc(t) dt s.t. (10)

	// dynamics and constraints given by (4)–(9)

	// Er(t f ) − q≤d∗ P1 − q. (11)

	// The glide slope constraint (12) ensures that the trajectory to the target cannot be too shallow or go subsurface.
	// X = (r, r˙) ∈ R6 : ˙r ≤ Vmax, E r − r(t f ) −cT r − r(t f ) ≤ 0 (12)

	// c  e1 tan γgs, γgs ∈ (0,π/2). (13)

	// Define constants and initial conditions
	const (
		rho1 = 0.1
		rho2 = 1.0
		alpha = 0.1
		m0 = 1000.0
		mf = 100.0
		tf = 100.0
	)

	var (
		x = [6]float64{0, 0, 0, 0, 0, 0}
		Tc = [3]float64{0, 0, 0}
		m = m0
	)

	// Define the dynamics and constraints
	A := func(omega [3]float64) [6][6]float64 {
		return [6][6]float64{
			{0, 0, 0, 1, 0, 0},
			{0, 0, 0, 0, 1, 0},
			{0, 0, 0, 0, 0, 1},
			{0, 0, 0, 0, omega[2], -omega[1]},
			{0, 0, 0, -omega[2], 0, omega[0]},
			{0, 0, 0, omega[1], -omega[0], 0},
		}
	}

	B := [6][3]float64{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}

	g := [3]float64{0, 0, -9.81}

	// Define the optimization problem
	for t := 0.0; t <= tf; t += 0.1 {
		// Update the state
		xDot := [6]float64{}
		for i := 0; i < 6; i++ {
			for j := 0; j < 6; j++ {
				xDot[i] += A([3]float64{0, 0, 0})[i][j] * x[j]
			}
			for j := 0; j < 3; j++ {
				xDot[i] += B[i][j] * (g[j] + Tc[j]/m)
			}
		}

		for i := 0; i < 6; i++ {
			x[i] += xDot[i] * 0.1
		}

		// Update the mass
		mDot := -alpha * math.Sqrt(Tc[0]*Tc[0]+Tc[1]*Tc[1]+Tc[2]*Tc[2])
		m += mDot * 0.1

		// Check constraints
		if m < m0-mf {
			break
		}
		if x[2] < 0 {
			break
		}
		if math.Sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]) > 0 {
			break
		}
	}
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
