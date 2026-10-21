package topobuild

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file holds the face-normal formula behind the public Face.NormalAt, so
// root and internal builders read one normal. Its proofs live in
// internal/surfacenormal.

// NormalAt returns the face's outward normal at p. Its bound combines the
// arithmetic proof for the normal of the face's tagged surface
// (surfacenormal) and any proven departure of the surface actually carried
// from that tag (topo.Face.NormalBound). A revolve wall's or cap's bound is
// instead proven against the surface its record denotes (topo.Face.Denoted),
// which covers its tag's departure and the arm's arithmetic at once. It is
// Exact only when the whole bound is zero. A point that gives the surface no
// direction is decaderr.ErrDegenerate. A reading whose own enclosure cannot
// separate the direction from zero is decaderr.ErrUnsupported.
//
// The five analytic variants — Plane, Cylinder, Cone, Sphere and Torus — are
// the whole set this answers for. Any other tagged surface is
// decaderr.ErrUnsupported: a NURBSSurface for the reason agent-docs/spline-design.md
// §7 owns, and a Faceted face — the tag every boolean-produced face carries —
// because its answer waits on the faceted certificate stage
// (agent-docs/payload-verification-design.md §5.4, §13). f must not be nil.
func NormalAt(f *topo.Face, p r3.Vec) (measurement.VecMeasurement, error) {
	switch s := f.Surface.(type) {
	case surfacegeom.Plane:
		n := s.Frame.N()
		return normalMeasurement(f, p, n, func() (float64, surfacenormal.Status) {
			return surfacenormal.PlaneAllow(s.Frame, n)
		}, "the frame of this plane names no direction")
	case surfacegeom.Cylinder:
		rel := p.Sub(s.Origin)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		dir, ok := radial.Normalize()
		if !ok {
			return measurement.VecMeasurement{}, fmt.Errorf(`%w: a point on the cylinder axis has no normal`, decaderr.ErrDegenerate)
		}
		return normalMeasurement(f, p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.AxialAllow(p, s.Origin, s.Axis, dir)
		}, "a point on the cylinder axis has no normal")
	case surfacegeom.Cone:
		rel := p.Sub(s.Origin)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		dir, ok := radial.Normalize()
		if !ok {
			return measurement.VecMeasurement{}, fmt.Errorf(`%w: the cone apex has no normal`, decaderr.ErrDegenerate)
		}
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			return measurement.VecMeasurement{}, fmt.Errorf(`decad: a cone's half angle is not an angle: %w`, err)
		}
		// The wall leans outward by the half angle along the growth axis, so
		// the geometric normal tilts against it by the same angle.
		n := dir.Scale(math.Cos(half)).Sub(s.Axis.Scale(math.Sin(half)))
		return normalMeasurement(f, p, n, func() (float64, surfacenormal.Status) {
			return surfacenormal.ConeAllow(p, s.Origin, s.Axis, half, n)
		}, "the cone apex has no normal")
	case surfacegeom.Sphere:
		dir, ok := p.Sub(s.Center).Normalize()
		if !ok {
			return measurement.VecMeasurement{}, fmt.Errorf(`%w: the sphere center has no normal`, decaderr.ErrDegenerate)
		}
		return normalMeasurement(f, p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.RadialAllow(p, s.Center, dir)
		}, "the sphere center has no normal")
	case surfacegeom.Torus:
		major, err := s.Major.In(units.Millimeter)
		if err != nil {
			return measurement.VecMeasurement{}, fmt.Errorf(`decad: a torus's major radius is not a length: %w`, err)
		}
		rel := p.Sub(s.Center)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		rdir, ok := radial.Normalize()
		if !ok {
			return measurement.VecMeasurement{}, fmt.Errorf(`%w: a point on the torus axis has no normal`, decaderr.ErrDegenerate)
		}
		tube := s.Center.Add(rdir.Scale(major))
		dir, ok := p.Sub(tube).Normalize()
		if !ok {
			return measurement.VecMeasurement{}, fmt.Errorf(`%w: the tube center has no normal`, decaderr.ErrDegenerate)
		}
		return normalMeasurement(f, p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.TorusAllow(p, s.Center, s.Axis, major, dir)
		}, "the tube center has no normal")
	default:
		return measurement.VecMeasurement{}, fmt.Errorf(`%w: this evaluator computes normals for its own analytic faces only`, decaderr.ErrUnsupported)
	}
}

// normalMeasurement publishes one arm's computed geometric direction under
// the face's own outward sign. tagAllow is the arm's proof against its tagged
// surface, and the face's own departure from its tag (NormalBound) composes
// with it by triangle inequality. A face carrying its denoted surface is
// proven against that instead: the comparison is with the surface the face
// carries, so it covers the departure NormalBound states, and neither
// tagAllow nor NormalBound enters. A retained face adds its HeldNormalBound
// on top, for the facets' displacement from the true patch. The sign is exact, so it never changes the
// bound.
func normalMeasurement(f *topo.Face, p, dir r3.Vec, tagAllow func() (float64, surfacenormal.Status), degenerate string) (measurement.VecMeasurement, error) {
	if f.Reversed {
		dir = dir.Scale(-1)
	}
	var allow float64
	var st surfacenormal.Status
	departure := 0.0
	if f.Denoted != nil {
		allow, st = f.Denoted.Allow(p, dir, f.Reversed)
		departure = f.HeldNormalBound
	} else {
		allow, st = tagAllow()
		departure = f.NormalBound
	}
	switch st {
	case surfacenormal.Zero:
		return measurement.VecMeasurement{}, fmt.Errorf(`%w: %s`, decaderr.ErrDegenerate, degenerate)
	case surfacenormal.Unproven:
		return measurement.VecMeasurement{}, fmt.Errorf(`%w: this normal's own direction is not proven away from zero, so no bound covers it`, decaderr.ErrUnsupported)
	}
	if allow != 0 || departure != 0 {
		allow = math.Nextafter(allow+departure, math.Inf(1))
	}
	return measurement.VecMeasurement{
		Value:     dir,
		Exactness: measurement.ExactnessOf(allow),
		Bound:     units.Scalar(allow),
	}, nil
}
