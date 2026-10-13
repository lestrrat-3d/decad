package coil

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// Magnitude is Table CS row CS4 over one value: its kind, its
// finiteness, then its sign. Zero is ErrDegenerate rather than a reduction:
// a zero pitch names a Revolve, and a zero turn count names no solid.
func Magnitude(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, error) {
	if v.Kind() != kind {
		return 0, fmt.Errorf(`%w: %s must be a %s, got %s`, decaderr.ErrUnitKind, what, kind, v.Kind())
	}
	m, err := v.In(unit)
	if err != nil || math.IsNaN(m) || math.IsInf(m, 0) {
		return 0, fmt.Errorf(`%w: %s is not finite`, decaderr.ErrNotFinite, what)
	}
	if m <= 0 {
		if what == "the coil pitch" {
			return 0, fmt.Errorf(`%w: %s must be positive, got %s; a zero pitch is a Revolve`, decaderr.ErrDegenerate, what, v)
		}
		return 0, fmt.Errorf(`%w: %s must be positive, got %s`, decaderr.ErrDegenerate, what, v)
	}
	return m, nil
}

// Preflight is Table CS row CS8, decided from counts alone before any
// allocation. The station and facet ceilings and the stations per turn are
// the caller's.
func Preflight(turns float64, vertices, holes int, perTurn, maxStations, maxFacets int64) error {
	n, ok := StationCount(proofarith.FloatRat(turns), perTurn)
	if !ok || n > maxStations {
		return fmt.Errorf(`%w: a coil of %v turns needs more than the %d-station ceiling at %d stations per turn`,
			decaderr.ErrUnsupported, turns, maxStations, perTurn)
	}
	walls := 2 * n * int64(vertices)
	caps := 2 * int64(vertices+2*holes-2)
	if walls+caps > maxFacets {
		return fmt.Errorf(`%w: the coil's %d triangles exceed the %d-triangle ceiling`, decaderr.ErrUnsupported, walls+caps, maxFacets)
	}
	return nil
}
