package decad

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// revolvedSectionRing shares every angular station across the section's
// recorded faces. An on-axis corner has one vertex rather than a full ring.
type revolvedSectionRing struct {
	angles []float64
	rings  [][]int
	groups []int
}

func (r revolvedSectionRing) vertex(span, station int) int {
	if len(r.rings[span]) == 1 {
		return r.rings[span][0]
	}
	return r.rings[span][station]
}

// buildRevolvedSectionRing meshes every authenticated meridian span except
// the axial span and the root-face intervals replaced by admitted sectors.
// The returned bound includes circular chording and held trig arithmetic.
// A caller may refine angles and retry when the bound exceeds its allowance.
func buildRevolvedSectionRing(mesh *joinedMesh, section revolveSectionCertificate,
	angles []float64, sectors [][2]float64, maxSag float64) (revolvedSectionRing, float64, error) {
	var empty revolvedSectionRing
	spans := section.spans
	if mesh == nil || len(spans) < 4 || len(angles) < 3 ||
		section.axisSpan < 0 || section.axisSpan >= len(spans) ||
		section.rootSpan < 0 || section.rootSpan >= len(spans) ||
		section.rootSpan == section.axisSpan ||
		maxSag <= 0 || proofbound.IsNonFinite(maxSag) {
		return empty, 0, fmt.Errorf("%w: revolved section ring has invalid inputs", ErrUnsupported)
	}
	if spans[section.axisSpan].start.V != 0 || spans[section.axisSpan].end.V != 0 ||
		spans[section.rootSpan].start.V <= 0 || spans[section.rootSpan].end.V <= 0 {
		return empty, 0, fmt.Errorf("%w: revolved section has no finite root ring", ErrUnsupported)
	}
	for i, span := range spans {
		if span.end != spans[(i+1)%len(spans)].start ||
			span.start.V < 0 || proofbound.IsNonFinite(span.start.U) ||
			proofbound.IsNonFinite(span.start.V) {
			return empty, 0, fmt.Errorf("%w: revolved section walk is incomplete", ErrUnsupported)
		}
	}
	for i, angle := range angles {
		if proofbound.IsNonFinite(angle) || (i != 0 && !(angles[i-1] < angle)) {
			return empty, 0, fmt.Errorf("%w: revolved ring angles are unordered", ErrUnsupported)
		}
	}
	span := new(big.Rat).Sub(proofarith.FloatRat(angles[len(angles)-1]),
		proofarith.FloatRat(angles[0]))
	if span.Cmp(proofbound.TwoPiInterval().Lo) >= 0 {
		return empty, 0, fmt.Errorf("%w: revolved ring covers more than one turn", ErrUnsupported)
	}
	startVerts, startTris := len(mesh.Verts), len(mesh.Tris)
	startSources, startGroups := len(mesh.Src), len(mesh.Groups)
	complete := false
	defer func() {
		if !complete {
			mesh.Verts = mesh.Verts[:startVerts]
			mesh.Tris = mesh.Tris[:startTris]
			mesh.Src = mesh.Src[:startSources]
			mesh.Groups = mesh.Groups[:startGroups]
		}
	}()

	ring := revolvedSectionRing{angles: append([]float64(nil), angles...),
		rings: make([][]int, len(spans))}
	cosHeld, sinHeld := make([]float64, len(angles)), make([]float64, len(angles))
	cosIv, sinIv := make([]proofbound.RatInterval, len(angles)),
		make([]proofbound.RatInterval, len(angles))
	for i, angle := range angles {
		var ok bool
		sinIv[i], cosIv[i], ok = proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return empty, 0, fmt.Errorf("%w: revolved ring angle has no trigonometric bound", ErrUnsupported)
		}
		cosHeld[i], sinHeld[i] = math.Cos(angle), math.Sin(angle)
	}
	maxRadius, maxRound := 0.0, 0.0
	for i, span := range spans {
		p := span.start
		if p.V == 0 {
			ring.rings[i] = []int{len(mesh.Verts)}
			mesh.Verts = append(mesh.Verts, r3.Vec{X: p.U})
			continue
		}
		maxRadius = max(maxRadius, p.V)
		ring.rings[i] = make([]int, len(angles))
		radius := proofarith.FloatRat(p.V)
		for j := range angles {
			held := r3.Vec{X: p.U, Y: p.V * cosHeld[j], Z: p.V * sinHeld[j]}
			if !proofbound.FiniteVec(held) {
				return empty, 0, fmt.Errorf("%w: revolved ring vertex is non-finite", ErrUnsupported)
			}
			yError := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv[j], radius), held.Y)
			zError := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv[j], radius), held.Z)
			maxRound = max(maxRound, proofbound.Radius3D(max(yError, zError)))
			ring.rings[i][j] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, held)
		}
	}
	if maxRadius <= 0 {
		return empty, 0, fmt.Errorf("%w: revolved section has no radial span", ErrUnsupported)
	}

	groupForSpan := make([]int, len(spans))
	for i, span := range spans {
		groupForSpan[i] = -1
		if i == section.axisSpan {
			continue
		}
		if span.face == nil || len(span.origins) == 0 {
			return empty, 0, fmt.Errorf("%w: revolved section face is missing", ErrUnsupported)
		}
		_, planar := span.face.n.Surface.(surfacegeom.Plane)
		groupForSpan[i] = len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetGroup{
			Origins: append([]provenance.Ref(nil), span.origins...),
			Surface: span.face.n.Surface, Denoted: span.face.n.Denoted,
			Reversed: span.face.n.Reversed, Planar: planar,
		})
	}
	ring.groups = groupForSpan

	maxStep := 0.0
	for j := range angles {
		k := (j + 1) % len(angles)
		step := new(big.Rat)
		if k == 0 {
			step.Add(proofbound.TwoPiInterval().Hi,
				new(big.Rat).Sub(proofarith.FloatRat(angles[0]), proofarith.FloatRat(angles[j])))
		} else {
			step.Sub(proofarith.FloatRat(angles[k]), proofarith.FloatRat(angles[j]))
		}
		stepUp := proofbound.RatFloatUp(step)
		if step.Sign() <= 0 || proofbound.IsNonFinite(stepUp) {
			return empty, 0, fmt.Errorf("%w: revolved ring has an invalid interval", ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
		for i, span := range spans {
			if i == section.axisSpan {
				continue
			}
			if i == section.rootSpan && k != 0 && sectionSectorCovered(angles[j], angles[k], sectors) {
				continue
			}
			a, b := ring.vertex(i, j), ring.vertex((i+1)%len(spans), j)
			c, d := ring.vertex((i+1)%len(spans), k), ring.vertex(i, k)
			group := groupForSpan[i]
			switch {
			case span.start.V == 0:
				mesh.Add([3]int{a, b, c}, group)
			case span.end.V == 0:
				mesh.Add([3]int{a, b, d}, group)
			default:
				mesh.Add([3]int{a, b, c}, group)
				mesh.Add([3]int{a, c, d}, group)
			}
		}
	}
	sag := proofbound.AbsSumUpper(proofbound.ProductUpper(maxRadius,
		proofbound.ProductUpper(maxStep, maxStep)/8), maxRound)
	if proofbound.IsNonFinite(sag) || sag > maxSag {
		return empty, 0, fmt.Errorf("%w: revolved ring exceeds its chord allowance", ErrUnsupported)
	}
	complete = true
	return ring, sag, nil
}

func sectionSectorCovered(lo, hi float64, sectors [][2]float64) bool {
	for _, sector := range sectors {
		if lo >= sector[0] && hi <= sector[1] {
			return true
		}
	}
	return false
}
