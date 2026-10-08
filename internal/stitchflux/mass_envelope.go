package stitchflux

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MassEnvelopeFace carries the readings needed to bound a curved face's
// placement allowance. Rims holds the first edge of each nonempty loop.
type MassEnvelopeFace struct {
	Surface         surfacegeom.Surface
	Area, AreaBound float64
	Vertices        []r3.Vec
	Rims            []CircleRim
	FirstRim        *CircleRim
}

// MassEnvelope bounds the area and farthest surface coordinate after every
// face has passed the curved flux admission gate.
func MassEnvelope(faces []MassEnvelopeFace, anchor r3.Vec) (float64, float64) {
	areaUpper, coordUpper := 0.0, 0.0
	for _, f := range faces {
		areaUpper = proofbound.AbsSumUpper(areaUpper, f.Area, f.AreaBound)
		for _, vertex := range f.Vertices {
			coordUpper = math.Max(coordUpper, vertex.Sub(anchor).Len())
		}
		switch surf := f.Surface.(type) {
		case surfacegeom.Cylinder:
			// A seam vertex alone need not be the farthest wall point.
			// Both rims share one radius. CircleRadius carries the proven
			// value and bound; the tagged Cylinder.Radius alone does not.
			if f.FirstRim != nil {
				coordUpper = addRimRadius(coordUpper, *f.FirstRim)
			}
		case surfacegeom.Cone:
			// A ruled wall reaches its farthest point on a rim circle.
			// Cone rims may have different radii, so both need a margin.
			for _, rim := range f.Rims {
				coordUpper = addRimRadius(coordUpper, rim)
			}
		case surfacegeom.Sphere:
			// A complete spherical face has no loop vertices to scan.
			// Its center distance plus the radius derived from the face's
			// proven area bounds every point on the sphere.
			if radius, err := SphereRadius(f.Area, f.AreaBound); err == nil {
				coordUpper = proofbound.AbsSumUpper(coordUpper, surf.Center.Sub(anchor).Len(), radius.Value, radius.Bound)
			}
		case surfacegeom.Torus:
			// Every torus point is within Major+Minor of its center.
			// Flux admission has already proved positive radii and a
			// coordinate-aligned axis before these tag values are read.
			if major, err := surf.Major.In(units.Millimeter); err == nil {
				if minor, err := surf.Minor.In(units.Millimeter); err == nil {
					coordUpper = proofbound.AbsSumUpper(coordUpper, surf.Center.Sub(anchor).Len(), major+minor)
				}
			}
		}
	}
	return areaUpper, coordUpper
}

func addRimRadius(coordUpper float64, rim CircleRim) float64 {
	radius, err := CircleRadius(rim.Length, rim.LengthBound, rim.LengthUnbounded)
	if err != nil {
		return coordUpper
	}
	return proofbound.AbsSumUpper(coordUpper, radius.Value, radius.Bound)
}
