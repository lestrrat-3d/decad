package breprecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// AxisMap names the source coordinate behind each placed coordinate.
// Its signs also map a placed box side back to the source side for face roles.
type AxisMap struct {
	source [3]int
	sign   [3]int
}

// PlacedGeometry re-expresses the canonical box and through-y bore under an
// exact signed-axis isometry. Each published coordinate is compared with its
// rational image, so the rebuilt corners still denote the rigid image of the
// original body. Other rigid motions retain the bounded refusal in §4.3b.
func (p AllEdgeChamferPayload) PlacedGeometry() (AllEdgeChamferPayload, AxisMap, error) {
	basis := p.Xform.Basis()
	images := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	axisMap := AxisMap{source: [3]int{-1, -1, -1}}
	for sourceAxis, image := range images {
		components := [3]float64{image.X, image.Y, image.Z}
		placedAxis := -1
		for axis, component := range components {
			if component == 0 {
				continue
			}
			if component != 1 && component != -1 || placedAxis >= 0 || axisMap.source[axis] >= 0 {
				return AllEdgeChamferPayload{}, AxisMap{},
					fmt.Errorf(`%w: the all-edge chamfer needs a signed-axis placement`, decaderr.ErrUnsupported)
			}
			placedAxis = axis
			axisMap.source[axis] = sourceAxis
			axisMap.sign[axis] = int(component)
		}
		if placedAxis < 0 {
			return AllEdgeChamferPayload{}, AxisMap{},
				fmt.Errorf(`%w: the all-edge chamfer needs a signed-axis placement`, decaderr.ErrUnsupported)
		}
	}
	if axisMap.source[1] != 1 {
		return AllEdgeChamferPayload{}, AxisMap{},
			fmt.Errorf(`%w: the placed all-edge chamfer bore must stay along y`, decaderr.ErrUnsupported)
	}
	translation := p.Xform.Translation()
	shift := [3]float64{translation.X, translation.Y, translation.Z}
	placed := p
	placed.Xform = r3.Identity()
	for axis := range 3 {
		sourceAxis := axisMap.source[axis]
		sign := axisMap.sign[axis]
		for sourceSide, original := range p.Box[sourceAxis] {
			var point [3]float64
			point[sourceAxis] = original
			image := p.Xform.Apply(vec(point))
			coordinate := [3]float64{image.X, image.Y, image.Z}[axis]
			if !finiteExactImage(coordinate, float64(sign)*original, shift[axis]) {
				return AllEdgeChamferPayload{}, AxisMap{},
					fmt.Errorf(`%w: a placed all-edge chamfer box endpoint rounded`, decaderr.ErrUnsupported)
			}
			placed.Box[axis][axisMap.sourceSide(axis, sourceSide)] = coordinate
		}
	}
	center := p.Xform.Apply(r3.NewVec(p.Center[0], 0, p.Center[1]))
	centerImage := [3]float64{center.X, center.Y, center.Z}
	for _, axis := range []int{0, 2} {
		sourceAxis := axisMap.source[axis]
		original := p.Center[0]
		if sourceAxis == 2 {
			original = p.Center[1]
		}
		if !finiteExactImage(centerImage[axis], float64(axisMap.sign[axis])*original, shift[axis]) {
			return AllEdgeChamferPayload{}, AxisMap{},
				fmt.Errorf(`%w: the placed all-edge chamfer bore center rounded`, decaderr.ErrUnsupported)
		}
		placed.Center[axis/2] = centerImage[axis]
	}
	if !placed.AdmitBox() || !placed.ExactCoordinates() {
		return AllEdgeChamferPayload{}, AxisMap{},
			fmt.Errorf(`%w: a placed all-edge chamfer corner is not exactly representable`, decaderr.ErrUnsupported)
	}
	return placed, axisMap, nil
}

func finiteExactImage(held, signedSource, shift float64) bool {
	return !math.IsNaN(held) && !math.IsInf(held, 0) && rationalSumEquals(signedSource, shift, held)
}

func (m AxisMap) sourceSide(placedAxis, placedSide int) int {
	if m.sign[placedAxis] < 0 {
		return 1 - placedSide
	}
	return placedSide
}

func (m AxisMap) faceRole(index int) string {
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
