package decad

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// bevelBlankSource is the six-line, unplaced axial section whose middle
// generator is the tooth's seating face. Its points are the recorded Sketch
// values, not values reconstructed from the rounded public Cone tag.
type bevelBlankSource struct {
	toeAxis, heelAxis Point2
	toeOuter, toeRoot Point2
	ded, heelOuter    Point2
	edgeIndex         [6]int
	rootFace          *Face
	rootCone          Cone
	// coneGap bounds the radial difference between the recorded straight
	// meridian and the public cone across the entire finite root segment.
	coneGap float64
}

// pointSourceCurrent checks the source identity held when LoftFromPoint
// recorded the profile. Certificate getters are checked separately at use.
func pointSourceCurrent(record *pointSectionRecord) bool {
	if record == nil || record.source == nil {
		return false
	}
	src := record.source
	if src.sketch == nil || src.profile == nil || src.profile.Sketch() != src.sketch ||
		src.profile.IsStale() || src.sketch.Revision() != src.revision ||
		src.profile.Revision() != src.revision ||
		len(src.entities) != len(record.profile.Outer.Segments) ||
		len(src.profile.Outer) != len(src.entities) {
		return false
	}
	if _, _, _, err := recordProfile(src.sketch, src.profile); err != nil {
		return false
	}
	frame, err := src.sketch.Plane().Frame()
	if err != nil || frame.Origin() != src.frame.Origin() ||
		frame.U() != src.frame.U() || frame.V() != src.frame.V() {
		return false
	}
	for i, entity := range src.entities {
		if entity == nil || src.profile.Outer[i].Entity != entity {
			return false
		}
	}
	return true
}

// The curved join reads the recorded finite meridian directly. The public
// root cone may have a rounded axial origin; the matcher charges its residual.
func exactSpiralBlank(b *Body) (*bevelBlankSource, bool) {
	return matchBevelBlankSection(b)
}

func matchBevelBlankSection(b *Body) (*bevelBlankSource, bool) {
	rp, ok := b.payload.(revolvePayload)
	if !ok || !rp.full || rp.surfaceResult || rp.sectionDelta != 0 ||
		rp.xform != r3.Identity() || len(rp.profile.Holes) != 0 ||
		len(rp.profile.Outer.Segments) != 6 ||
		rp.frame.Origin() != (r3.Vec{}) || rp.frame.U() != (r3.Vec{X: 1}) ||
		rp.frame.V() != (r3.Vec{Y: 1}) ||
		rp.ax.AU != 0 || rp.ax.AV != 0 || rp.ax.DU != 1 || rp.ax.DV != 0 ||
		rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 {
		return nil, false
	}
	segments := make([]lineSeg, 6)
	points := map[Point2]struct{}{}
	for i, segment := range rp.profile.Outer.Segments {
		line, lineOK := segment.(lineSeg)
		if !lineOK || !((line.TStart == 0 && line.TEnd == 1) ||
			(line.TStart == 1 && line.TEnd == 0)) ||
			!proofbound.FiniteVec(r3.Vec{X: line.Start.U, Y: line.Start.V}) ||
			!proofbound.FiniteVec(r3.Vec{X: line.End.U, Y: line.End.V}) ||
			line.Start == line.End {
			return nil, false
		}
		segments[i] = line
		points[line.Start], points[line.End] = struct{}{}, struct{}{}
	}
	if len(points) != 6 {
		return nil, false
	}
	var axis, outer []Point2
	for point := range points {
		if point.U <= 0 || point.V < 0 {
			return nil, false
		}
		if point.V == 0 {
			axis = append(axis, point)
		} else {
			outer = append(outer, point)
		}
	}
	if len(axis) != 2 || len(outer) != 4 {
		return nil, false
	}
	if axis[0].U > axis[1].U {
		axis[0], axis[1] = axis[1], axis[0]
	}
	result := &bevelBlankSource{toeAxis: axis[0], heelAxis: axis[1]}
	interior := make([]Point2, 0, 2)
	for _, point := range outer {
		switch point.U {
		case result.toeAxis.U:
			if result.toeOuter != (Point2{}) {
				return nil, false
			}
			result.toeOuter = point
		case result.heelAxis.U:
			if result.heelOuter != (Point2{}) {
				return nil, false
			}
			result.heelOuter = point
		default:
			interior = append(interior, point)
		}
	}
	if len(interior) != 2 || result.toeOuter == (Point2{}) ||
		result.heelOuter == (Point2{}) {
		return nil, false
	}
	if interior[0].U > interior[1].U {
		interior[0], interior[1] = interior[1], interior[0]
	}
	result.toeRoot, result.ded = interior[0], interior[1]
	if !(result.toeRoot.U < result.toeAxis.U &&
		result.toeAxis.U < result.ded.U && result.ded.U < result.heelAxis.U) {
		return nil, false
	}
	cycle := [6]Point2{result.toeAxis, result.heelAxis, result.heelOuter,
		result.ded, result.toeRoot, result.toeOuter}
	rootIndex := -1
	var seen [6]bool
	for i, line := range segments {
		matched := false
		for j := range cycle {
			a, z := cycle[j], cycle[(j+1)%len(cycle)]
			if line.Start == a && line.End == z || line.Start == z && line.End == a {
				if seen[j] {
					return nil, false
				}
				seen[j] = true
				matched = true
				result.edgeIndex[j] = i
				if j == 3 {
					rootIndex = i
				}
				break
			}
		}
		if !matched {
			return nil, false
		}
	}
	if rootIndex < 0 {
		return nil, false
	}
	role := fmt.Sprintf("side(0,%d)", rootIndex)
	for _, face := range b.Faces() {
		found := false
		for _, origin := range face.Origins() {
			found = found || origin.Role == role
		}
		if !found {
			continue
		}
		cone, coneOK := face.Surface().(Cone)
		if !coneOK || cone.Axis != (r3.Vec{X: 1}) ||
			cone.Origin.Y != 0 || cone.Origin.Z != 0 ||
			!proofbound.FiniteVec(cone.Origin) {
			return nil, false
		}
		radius, radiusErr := cone.Radius.In(units.Millimeter)
		angle, angleErr := cone.HalfAngle.In(units.Radian)
		if radiusErr != nil || angleErr != nil || radius != 0 ||
			!proofbound.FiniteVec(r3.Vec{X: angle}) || angle <= 0 || angle >= math.Pi/2 {
			return nil, false
		}
		slope := math.Tan(angle)
		if !proofbound.FiniteVec(r3.Vec{X: slope}) || slope <= 0 {
			return nil, false
		}
		result.rootFace, result.rootCone = face, cone
		result.coneGap = max(bevelMeridianResidualAt(result.toeRoot, slope, cone.Origin.X),
			bevelMeridianResidualAt(result.ded, slope, cone.Origin.X))
		return result, !proofbound.IsNonFinite(result.coneGap)
	}
	return nil, false
}

// A straight meridian and a cone generator differ linearly in station, so
// the endpoint maximum bounds every point between them without sampling.
func bevelMeridianResidualAt(point Point2, slope, originX float64) float64 {
	r := new(big.Rat).Sub(proofarith.FloatRat(point.V),
		new(big.Rat).Mul(proofarith.FloatRat(slope),
			new(big.Rat).Sub(proofarith.FloatRat(point.U), proofarith.FloatRat(originX))))
	if r.Sign() < 0 {
		r.Neg(r)
	}
	return proofbound.RatFloatUp(r)
}

func (source *bevelBlankSource) meridianEquation() (*big.Rat, *big.Rat) {
	x0, x1 := proofarith.FloatRat(source.toeRoot.U), proofarith.FloatRat(source.ded.U)
	r0, r1 := proofarith.FloatRat(source.toeRoot.V), proofarith.FloatRat(source.ded.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	return m, b
}
