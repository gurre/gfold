package gfold_test

import (
	"testing"

	"github.com/gurre/gfold"
)

func TestNumericalExample(t *testing.T) {
	g := gfold.New(gfold.WithMinimalFuelUse())

	g.SoftLandTrajectory()
}
