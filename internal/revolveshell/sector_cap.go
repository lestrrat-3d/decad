package revolveshell

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// revolveFarRootPoint maps a direction on the authenticated root cone back
// to the point-loft source plane. The cone apex may lie anywhere on its axis.
func revolveFarRootPoint(frame r3.Frame, cone surfacegeom.Cone, slope, angle float64) (r3.Vec, bool) {
	direction := r3.Vec{X: 1, Y: slope * math.Cos(angle), Z: slope * math.Sin(angle)}
	return rootGeneratorPoint(frame, cone.Origin, direction)
}

// revolveFarRootPointAbout is revolveFarRootPoint for a general certificate.
// Its generator W + s·(E0·cos θ + E1·sin θ) starts at the reported cone's
// apex and reads the blank's own held basis, the frame the angle θ is
// measured in (agent-docs/revolve-axis-design.md §3.7, §5.7). The point only
// places a cap station: the build snaps it to its ring vertex and charges
// the measured snap.
func revolveFarRootPointAbout(frame r3.Frame, general *facetedrecord.RevolveSectionAxis,
	angle float64) (r3.Vec, bool) {
	sin, cos := math.Sincos(angle)
	slope := general.Cone.Slope
	radial := general.E0.Scale(cos).Add(general.E1.Scale(sin))
	return rootGeneratorPoint(frame, general.Cone.Apex, general.W.Add(radial.Scale(slope)))
}

// rootGeneratorPoint intersects the ray from apex along direction with the
// plane of frame, and reports false unless the crossing lies ahead of apex
// and is finite.
func rootGeneratorPoint(frame r3.Frame, apex, direction r3.Vec) (r3.Vec, bool) {
	normal := frame.U().Cross(frame.V())
	den := normal.Dot(direction)
	if den == 0 {
		return r3.Vec{}, false
	}
	distance := normal.Dot(frame.Origin().Sub(apex)) / den
	point := apex.Add(direction.Scale(distance))
	return point, proofbound.FiniteVec(point) && distance > 0
}

// RevolveSectorCapBoundary closes the certified outside route with stations
// on the root face. Every root station has an exact matching angular index in
// the revolved section mesh. Angles are about the section's axis, in its
// AngleBasis.
func RevolveSectorCapBoundary(path []pointcone.SectorNode, section facetedrecord.RevolveSectionCertificate,
	frame r3.Frame, angles []float64, sector [2]float64) ([]pointcone.SectorNode, error) {
	if len(path) < 3 || len(angles) < 3 {
		return nil, fmt.Errorf("%w: sector cap has no closed source route", decaderr.ErrUnsupported)
	}
	basis := section.AngleBasis()
	nodes := append([]pointcone.SectorNode(nil), path...)
	for _, i := range []int{0, len(nodes) - 1} {
		nodes[i].RootBoundary = true
		nodes[i].Angle = basis.Angle(nodes[i].P) + 2*math.Pi
		if nodes[i].Angle < sector[0] || nodes[i].Angle > sector[1] {
			return nil, fmt.Errorf("%w: root contact is outside the angular sector", decaderr.ErrUnsupported)
		}
	}
	nodes[len(nodes)-1].GroupToNext = -1
	root := make([]float64, 0, len(angles))
	for _, angle := range angles {
		if sector[0] < angle && angle < sector[1] {
			root = append(root, angle)
		}
	}
	appendRoot := func(angle float64) error {
		var point r3.Vec
		var ok bool
		if section.General != nil {
			point, ok = revolveFarRootPointAbout(frame, section.General, angle)
		} else {
			point, ok = revolveFarRootPoint(frame, section.RootCone,
				section.ConeSection().Slope, angle)
		}
		if !ok {
			return fmt.Errorf("%w: root generator misses the source plane", decaderr.ErrUnsupported)
		}
		nodes = append(nodes, pointcone.SectorNode{P: point, GroupToNext: -1,
			Crossing: -1, RootBoundary: true, Angle: angle})
		return nil
	}
	if nodes[len(nodes)-1].Angle == sector[1] {
		for _, angle := range slices.Backward(root) {
			if err := appendRoot(angle); err != nil {
				return nil, err
			}
		}
	} else if nodes[len(nodes)-1].Angle == sector[0] {
		for _, angle := range root {
			if err := appendRoot(angle); err != nil {
				return nil, err
			}
		}
	} else {
		return nil, fmt.Errorf("%w: root contact does not close the sector", decaderr.ErrUnsupported)
	}
	return nodes, nil
}
