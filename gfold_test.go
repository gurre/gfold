package gfold_test

import (
	"testing"

	"github.com/gurre/gfold"
)

func TestNumericalExample(t *testing.T) {
	g := gfold.New(gfold.WithMinimalFuelUse())

	g.SoftLandTrajectory()
}

func TestMinimumFuel(t *testing.T) {
	g := gfold.New(gfold.WithMinimalFuelUse())

	thrustAngle := g.SoftLandTrajectory()

	if thrustAngle != 0 {
		t.Errorf("Expected thrust angle to be 0, but got %f", thrustAngle)
	}
}

func TestMinimumLandingError(t *testing.T) {
	g := gfold.New()

	thrustAngle := g.SoftLandTrajectory()

	if thrustAngle != 0 {
		t.Errorf("Expected thrust angle to be 0, but got %f", thrustAngle)
	}
}
