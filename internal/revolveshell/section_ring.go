package revolveshell

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// BuildRevolvedSectionRing meshes every authenticated meridian span except
// the axial span and the root-face intervals replaced by admitted sectors.
// The returned bound includes circular chording and held trig arithmetic.
// A caller may refine angles and retry when the bound exceeds its allowance.
func BuildRevolvedSectionRing(mesh *facetedrecord.JoinedMesh, section facetedrecord.RevolveSectionCertificate,
	angles []float64, sectors [][2]float64, maxSag float64) (facetedrecord.RevolvedSectionRing, float64, error) {
	var empty facetedrecord.RevolvedSectionRing
	spans := section.Spans
	if mesh == nil || len(spans) < 4 || len(angles) < 3 ||
		section.AxisSpan < 0 || section.AxisSpan >= len(spans) ||
		section.RootSpan < 0 || section.RootSpan >= len(spans) ||
		section.RootSpan == section.AxisSpan ||
		maxSag <= 0 || proofbound.IsNonFinite(maxSag) {
		return empty, 0, fmt.Errorf("%w: revolved section ring has invalid inputs", decaderr.ErrUnsupported)
	}
	if spans[section.AxisSpan].Start.V != 0 || spans[section.AxisSpan].End.V != 0 ||
		spans[section.RootSpan].Start.V <= 0 || spans[section.RootSpan].End.V <= 0 {
		return empty, 0, fmt.Errorf("%w: revolved section has no finite root ring", decaderr.ErrUnsupported)
	}
	for i, span := range spans {
		if span.End != spans[(i+1)%len(spans)].Start ||
			span.Start.V < 0 || proofbound.IsNonFinite(span.Start.U) ||
			proofbound.IsNonFinite(span.Start.V) {
			return empty, 0, fmt.Errorf("%w: revolved section walk is incomplete", decaderr.ErrUnsupported)
		}
	}
	for i, angle := range angles {
		if proofbound.IsNonFinite(angle) || (i != 0 && !(angles[i-1] < angle)) {
			return empty, 0, fmt.Errorf("%w: revolved ring angles are unordered", decaderr.ErrUnsupported)
		}
	}
	span := new(big.Rat).Sub(proofarith.FloatRat(angles[len(angles)-1]),
		proofarith.FloatRat(angles[0]))
	if span.Cmp(proofbound.TwoPiInterval().Lo) >= 0 {
		return empty, 0, fmt.Errorf("%w: revolved ring covers more than one turn", decaderr.ErrUnsupported)
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

	ring := facetedrecord.RevolvedSectionRing{Angles: append([]float64(nil), angles...),
		Rings: make([][]int, len(spans))}
	cosHeld, sinHeld := make([]float64, len(angles)), make([]float64, len(angles))
	cosIv, sinIv := make([]proofbound.RatInterval, len(angles)),
		make([]proofbound.RatInterval, len(angles))
	for i, angle := range angles {
		var ok bool
		sinIv[i], cosIv[i], ok = proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return empty, 0, fmt.Errorf("%w: revolved ring angle has no trigonometric bound", decaderr.ErrUnsupported)
		}
		cosHeld[i], sinHeld[i] = math.Cos(angle), math.Sin(angle)
	}
	maxRadius, maxRound := 0.0, 0.0
	for i, span := range spans {
		p := span.Start
		if p.V == 0 {
			ring.Rings[i] = []int{len(mesh.Verts)}
			mesh.Verts = append(mesh.Verts, r3.Vec{X: p.U})
			continue
		}
		maxRadius = max(maxRadius, p.V)
		ring.Rings[i] = make([]int, len(angles))
		radius := proofarith.FloatRat(p.V)
		for j := range angles {
			held := r3.Vec{X: p.U, Y: p.V * cosHeld[j], Z: p.V * sinHeld[j]}
			if !proofbound.FiniteVec(held) {
				return empty, 0, fmt.Errorf("%w: revolved ring vertex is non-finite", decaderr.ErrUnsupported)
			}
			yError := proofbound.IntervalFloatError(proofbound.IntervalScale(cosIv[j], radius), held.Y)
			zError := proofbound.IntervalFloatError(proofbound.IntervalScale(sinIv[j], radius), held.Z)
			maxRound = max(maxRound, proofbound.Radius3D(max(yError, zError)))
			ring.Rings[i][j] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, held)
		}
	}
	if maxRadius <= 0 {
		return empty, 0, fmt.Errorf("%w: revolved section has no radial span", decaderr.ErrUnsupported)
	}

	groupForSpan := make([]int, len(spans))
	for i, span := range spans {
		groupForSpan[i] = -1
		if i == section.AxisSpan {
			continue
		}
		if span.Face == nil || len(span.Origins) == 0 {
			return empty, 0, fmt.Errorf("%w: revolved section face is missing", decaderr.ErrUnsupported)
		}
		_, planar := span.Face.Surface.(surfacegeom.Plane)
		groupForSpan[i] = len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetedrecord.Group{
			Origins: append([]provenance.Ref(nil), span.Origins...),
			Surface: span.Face.Surface, Denoted: span.Face.Denoted,
			Reversed: span.Face.Reversed, Planar: planar,
		})
	}
	ring.Groups = groupForSpan

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
			return empty, 0, fmt.Errorf("%w: revolved ring has an invalid interval", decaderr.ErrUnsupported)
		}
		maxStep = max(maxStep, stepUp)
		for i, span := range spans {
			if i == section.AxisSpan {
				continue
			}
			if i == section.RootSpan && k != 0 && sectionSectorCovered(angles[j], angles[k], sectors) {
				continue
			}
			a, b := ring.Vertex(i, j), ring.Vertex((i+1)%len(spans), j)
			c, d := ring.Vertex((i+1)%len(spans), k), ring.Vertex(i, k)
			group := groupForSpan[i]
			switch {
			case span.Start.V == 0:
				mesh.Add([3]int{a, b, c}, group)
			case span.End.V == 0:
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
		return empty, 0, fmt.Errorf("%w: revolved ring exceeds its chord allowance", decaderr.ErrUnsupported)
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
