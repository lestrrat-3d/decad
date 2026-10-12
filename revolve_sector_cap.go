package decad

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// revolveFarRootPoint maps a direction on the authenticated root cone back
// to the point-loft source plane. The cone apex may lie anywhere on its axis.
func revolveFarRootPoint(frame r3.Frame, cone Cone, slope, angle float64) (r3.Vec, bool) {
	normal := frame.U().Cross(frame.V())
	direction := r3.Vec{X: 1, Y: slope * math.Cos(angle), Z: slope * math.Sin(angle)}
	den := normal.Dot(direction)
	if den == 0 {
		return r3.Vec{}, false
	}
	distance := normal.Dot(frame.Origin().Sub(cone.Origin)) / den
	point := cone.Origin.Add(direction.Scale(distance))
	return point, proofbound.FiniteVec(point) && distance > 0
}

// revolveSectorCapBoundary closes the certified outside route with stations
// on the root face. Every root station has an exact matching angular index in
// the revolved section mesh.
func revolveSectorCapBoundary(path []sectorPathNode, section revolveSectionCertificate,
	frame r3.Frame, angles []float64, sector [2]float64) ([]sectorPathNode, error) {
	if len(path) < 3 || len(angles) < 3 {
		return nil, fmt.Errorf("%w: sector cap has no closed source route", ErrUnsupported)
	}
	nodes := append([]sectorPathNode(nil), path...)
	for _, i := range []int{0, len(nodes) - 1} {
		nodes[i].rootBoundary = true
		nodes[i].angle = math.Atan2(nodes[i].p.Z, nodes[i].p.Y) + 2*math.Pi
		if nodes[i].angle < sector[0] || nodes[i].angle > sector[1] {
			return nil, fmt.Errorf("%w: root contact is outside the angular sector", ErrUnsupported)
		}
	}
	nodes[len(nodes)-1].groupToNext = -1
	root := make([]float64, 0, len(angles))
	for _, angle := range angles {
		if sector[0] < angle && angle < sector[1] {
			root = append(root, angle)
		}
	}
	appendRoot := func(angle float64) error {
		point, ok := revolveFarRootPoint(frame, section.rootCone,
			section.coneSection().Slope, angle)
		if !ok {
			return fmt.Errorf("%w: root generator misses the source plane", ErrUnsupported)
		}
		nodes = append(nodes, sectorPathNode{p: point, groupToNext: -1,
			crossing: -1, rootBoundary: true, angle: angle})
		return nil
	}
	if nodes[len(nodes)-1].angle == sector[1] {
		for _, angle := range slices.Backward(root) {
			if err := appendRoot(angle); err != nil {
				return nil, err
			}
		}
	} else if nodes[len(nodes)-1].angle == sector[0] {
		for _, angle := range root {
			if err := appendRoot(angle); err != nil {
				return nil, err
			}
		}
	} else {
		return nil, fmt.Errorf("%w: root contact does not close the sector", ErrUnsupported)
	}
	return nodes, nil
}
