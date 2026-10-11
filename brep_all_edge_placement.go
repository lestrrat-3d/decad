package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// allEdgeAxisMap names the source coordinate behind each placed coordinate.
// Its signs also map a placed box side back to the source side for face roles.
type allEdgeAxisMap struct {
	source [3]int
	sign   [3]int
}

// placedGeometry re-expresses the canonical box and through-y bore under an
// exact signed-axis isometry. Each published coordinate is compared with its
// rational image, so the rebuilt corners still denote the rigid image of the
// original body. Other rigid motions retain the bounded refusal in §4.3b.
func (p allEdgeChamferPayload) placedGeometry() (allEdgeChamferPayload, allEdgeAxisMap, error) {
	basis := p.xform.Basis()
	images := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	axisMap := allEdgeAxisMap{source: [3]int{-1, -1, -1}}
	for sourceAxis, image := range images {
		components := [3]float64{image.X, image.Y, image.Z}
		placedAxis := -1
		for axis, component := range components {
			if component == 0 {
				continue
			}
			if component != 1 && component != -1 || placedAxis >= 0 || axisMap.source[axis] >= 0 {
				return allEdgeChamferPayload{}, allEdgeAxisMap{},
					fmt.Errorf(`%w: the all-edge chamfer needs a signed-axis placement`, ErrUnsupported)
			}
			placedAxis = axis
			axisMap.source[axis] = sourceAxis
			axisMap.sign[axis] = int(component)
		}
		if placedAxis < 0 {
			return allEdgeChamferPayload{}, allEdgeAxisMap{},
				fmt.Errorf(`%w: the all-edge chamfer needs a signed-axis placement`, ErrUnsupported)
		}
	}
	if axisMap.source[1] != 1 {
		return allEdgeChamferPayload{}, allEdgeAxisMap{},
			fmt.Errorf(`%w: the placed all-edge chamfer bore must stay along y`, ErrUnsupported)
	}
	translation := p.xform.Translation()
	shift := [3]float64{translation.X, translation.Y, translation.Z}
	placed := p
	placed.xform = r3.Identity()
	for axis := range 3 {
		sourceAxis := axisMap.source[axis]
		sign := axisMap.sign[axis]
		for sourceSide, original := range p.box[sourceAxis] {
			var point [3]float64
			point[sourceAxis] = original
			image := p.xform.Apply(vec(point))
			coordinate := [3]float64{image.X, image.Y, image.Z}[axis]
			if !finiteExactImage(coordinate, float64(sign)*original, shift[axis]) {
				return allEdgeChamferPayload{}, allEdgeAxisMap{},
					fmt.Errorf(`%w: a placed all-edge chamfer box endpoint rounded`, ErrUnsupported)
			}
			placed.box[axis][axisMap.sourceSide(axis, sourceSide)] = coordinate
		}
	}
	center := p.xform.Apply(r3.NewVec(p.center[0], 0, p.center[1]))
	centerImage := [3]float64{center.X, center.Y, center.Z}
	for _, axis := range []int{0, 2} {
		sourceAxis := axisMap.source[axis]
		original := p.center[0]
		if sourceAxis == 2 {
			original = p.center[1]
		}
		if !finiteExactImage(centerImage[axis], float64(axisMap.sign[axis])*original, shift[axis]) {
			return allEdgeChamferPayload{}, allEdgeAxisMap{},
				fmt.Errorf(`%w: the placed all-edge chamfer bore center rounded`, ErrUnsupported)
		}
		placed.center[axis/2] = centerImage[axis]
	}
	if !placed.admitBox() || !placed.exactCoordinates() {
		return allEdgeChamferPayload{}, allEdgeAxisMap{},
			fmt.Errorf(`%w: a placed all-edge chamfer corner is not exactly representable`, ErrUnsupported)
	}
	return placed, axisMap, nil
}

func finiteExactImage(held, signedSource, shift float64) bool {
	return !math.IsNaN(held) && !math.IsInf(held, 0) && rationalSumEquals(signedSource, shift, held)
}

func (m allEdgeAxisMap) sourceSide(placedAxis, placedSide int) int {
	if m.sign[placedAxis] < 0 {
		return 1 - placedSide
	}
	return placedSide
}

func (m allEdgeAxisMap) faceRole(index int) string {
	if index < 6 {
		axis, side := index/2, index%2
		return fmt.Sprintf("face(%d)", 2*m.source[axis]+m.sourceSide(axis, side))
	}
	index -= 6
	axis, sideB, sideC := index/4, index%4/2, index%2
	b, c := (axis+1)%3, (axis+2)%3
	sourceAxis := m.source[axis]
	var sourceSides [3]int
	sourceSides[m.source[b]] = m.sourceSide(b, sideB)
	sourceSides[m.source[c]] = m.sourceSide(c, sideC)
	return fmt.Sprintf("chamfer(%d,%d,%d)", sourceAxis,
		sourceSides[(sourceAxis+1)%3], sourceSides[(sourceAxis+2)%3])
}
