package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderChamferSource is the already audited cap and side correspondence
// of an axial loft joined to an exact circular blank. The cap loops are CCW
// when viewed from +Z, including the start cap whose public coedges are CW.
type loftCylinderChamferSource struct {
	blank          *Body
	teeth          []*Body
	originalTeeth  []*Body
	axialBridge    []float64
	bridgeBound    float64
	topSourceLevel *big.Rat
	// Optional physical-height correction for a reflected, axially bridged
	// source tooth. The ring's shared held level stays unchanged.
	ringLevelShift []*big.Rat
	outer, inner   [2][]int
	capGroups      [2]int
	boreGroup      int
	blankRadius    float64
	boreRadius     float64
	boreBound      float64
	innerInset     float64
	outerExcess    float64
	rootCollar     float64
	z0, z1         float64
}

func loftCylinderChamferSourceOf(fp facetedPayload) (loftCylinderChamferSource, bool) {
	if !fp.contactAudited || fp.xform != r3.Identity() {
		return loftCylinderChamferSource{}, false
	}
	if r := fp.loftCylinderBore; r != nil {
		return loftCylinderChamferSource{
			blank: r.blank, teeth: r.proofTeeth,
			originalTeeth: r.teeth, axialBridge: r.axialBridge,
			outer: r.outer, inner: r.inner, capGroups: r.capGroups,
			boreGroup: r.wallGroup, blankRadius: r.blankRadius,
			boreRadius: r.boreRadius, boreBound: r.boreBound, innerInset: r.innerInset,
			outerExcess: r.outerExcess, rootCollar: r.rootCollar,
			z0: r.z0, z1: r.z1,
		}, true
	}
	if r := fp.loftCylinderJoin; r != nil {
		return loftCylinderChamferSource{
			blank: r.blank, teeth: r.proofTeeth,
			originalTeeth: r.teeth, axialBridge: r.axialBridge,
			outer: r.capOutlines, capGroups: r.capGroups,
			boreGroup: -1, blankRadius: r.radius,
			innerInset: r.innerInset, outerExcess: r.outerExcess,
			rootCollar: r.rootCollar, z0: r.z0, z1: r.z1,
		}, true
	}
	return loftCylinderChamferSource{}, false
}

// The cap audit proves the mesh boundary is exactly ring. Public edges may
// coalesce adjacent segments, so their endpoints must partition that whole
// boundary once in the cap's walk direction.
func loftCylinderChamferCapWalk(loop *topo.Loop, verts []r3.Vec, ring []int, bottom bool) bool {
	if loop == nil || !loop.Outer || len(loop.CoEdges) < 3 || len(loop.CoEdges) > len(ring) {
		return false
	}
	index := make(map[r3.Vec]int, len(ring))
	for i, vertex := range ring {
		if vertex < 0 || vertex >= len(verts) {
			return false
		}
		if _, duplicate := index[verts[vertex]]; duplicate {
			return false
		}
		index[verts[vertex]] = i
	}
	covered := 0
	for i, use := range loop.CoEdges {
		if coedgeStart(use) == nil || coedgeEnd(use) == nil ||
			coedgeEnd(use).Position != coedgeStart(loop.CoEdges[(i+1)%len(loop.CoEdges)]).Position {
			return false
		}
		a, okA := index[coedgeStart(use).Position]
		b, okB := index[coedgeEnd(use).Position]
		if !okA || !okB {
			return false
		}
		steps := (b - a + len(ring)) % len(ring)
		if bottom {
			steps = (a - b + len(ring)) % len(ring)
		}
		if steps == 0 {
			return false
		}
		covered += steps
		if covered > len(ring) {
			return false
		}
	}
	return covered == len(ring)
}

// tryLoftCylinderCapChamfer accepts exactly the two full outer cap loops of
// an authenticated, unplaced loft/cylinder join or its centered bore result.
// It never interprets an arbitrary faceted edge as an offset carrier.
func tryLoftCylinderCapChamfer(ctx context.Context, b *Body, fp facetedPayload,
	edges []*Edge, d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return nil, false, nil
	}
	if asym != nil {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for this loft/cylinder cap", ErrUnsupported)
	}
	if dDelta != 0 || d <= 0 || !proofbound.FiniteVec(r3.Vec{X: d, Y: source.z0, Z: source.z1}) {
		return nil, true, fmt.Errorf("%w: loft/cylinder cap setback has no exact millimeter magnitude", ErrUnsupported)
	}
	startExact := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(d))
	endExact := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(d))
	if startExact.Cmp(endExact) >= 0 || source.z0+d == source.z0 || source.z1-d == source.z1 {
		return nil, true, fmt.Errorf("%w: loft/cylinder cap bands reach each other or round onto a cap", ErrUnsupported)
	}
	if err := loftCylinderChamferBridgeSources(&source); err != nil {
		return nil, true, err
	}
	if source.topSourceLevel.Cmp(endExact) <= 0 {
		return nil, true, fmt.Errorf("%w: cap chamfer bridge reaches its side level", ErrUnsupported)
	}
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				if caps[0] != nil {
					return nil, true, fmt.Errorf("%w: loft/cylinder start cap is split", ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
				if caps[1] != nil {
					return nil, true, fmt.Errorf("%w: loft/cylinder end cap is split", ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, true, fmt.Errorf("%w: loft/cylinder result has no current cap faces", ErrUnsupported)
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	if len(selected) != len(edges) {
		return nil, true, fmt.Errorf("%w: loft/cylinder chamfer repeats an edge", ErrUnsupported)
	}
	for cap, face := range caps {
		if len(face.n.Loops) == 0 || !loftCylinderChamferCapWalk(
			face.n.Loops[0], fp.verts, source.outer[cap], cap == 0) {
			return nil, true, fmt.Errorf("%w: loft/cylinder cap %d has no outer loop", ErrUnsupported, cap)
		}
		for _, ce := range face.n.Loops[0].CoEdges {
			edge := ce.Edge
			if _, yes := selected[edge]; !yes {
				return nil, true, fmt.Errorf("%w: loft/cylinder chamfer needs both complete outer cap loops", ErrUnsupported)
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, true, fmt.Errorf("%w: loft/cylinder chamfer selection includes a bore or side edge", ErrUnsupported)
	}
	body, err := buildLoftCylinderCapChamfer(ctx, documentHandle(b.n.Doc), b.n.Doc.NextProducer, fp, source, d)
	return body, true, err
}

type loftCylinderInsetSamples struct {
	points              []r3.Vec
	lineStart, arcStart []int
	bound               float64
}

// loftCylinderInset chords the analytic offset of one authenticated held cap
// outline. Each source edge owns one offset line and its following corner arc.
func loftCylinderInset(ctx context.Context, fp facetedPayload, ring []int, d float64) (loftCylinderInsetSamples, error) {
	if len(ring) < 3 {
		return loftCylinderInsetSamples{}, fmt.Errorf("%w: cap outline has fewer than three vertices", ErrUnsupported)
	}
	segs := make([]curveSegment, len(ring))
	for i, index := range ring {
		next := ring[(i+1)%len(ring)]
		if index < 0 || next < 0 || index >= len(fp.verts) || next >= len(fp.verts) {
			return loftCylinderInsetSamples{}, fmt.Errorf("%w: cap outline has an invalid vertex", ErrUnsupported)
		}
		a, b := fp.verts[index], fp.verts[next]
		segs[i] = lineSeg{Start: sectionrecord.Point2{U: a.X, V: a.Y},
			End: sectionrecord.Point2{U: b.X, V: b.Y}, TStart: 0, TEnd: 1}
	}
	profile := profileRecord{Outer: loopRecord{Segments: segs}}
	budget := proofbound.NewWorkBudget(ctx)
	inset, err := offsetProfile(budget, profile, 1, d)
	if err != nil {
		return loftCylinderInsetSamples{}, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, profile, inset); err != nil {
		return loftCylinderInsetSamples{}, err
	}
	delta, err := offsetSectionDelta(budget, profile, 1, d, 0)
	if err != nil {
		return loftCylinderInsetSamples{}, err
	}
	const chord = 1e-4
	samples, err := tessellation.ChordLoop(ctx, inset.Outer, chord, d,
		freeform.NewFreeformWork(), nil, 0,
		func(w survey2d.SideWalk) (int, error) { return w.Segs[0], nil }, stationbound.ChordStationBound)
	if err != nil {
		return loftCylinderInsetSamples{}, err
	}
	out := loftCylinderInsetSamples{
		points:    make([]r3.Vec, len(samples.Samples)),
		lineStart: make([]int, len(ring)), arcStart: make([]int, len(ring)),
		bound: proofbound.AbsSumUpper(delta, samples.MaxSag),
	}
	for i := range out.arcStart {
		out.arcStart[i] = -1
	}
	for i, p := range samples.Samples {
		out.points[i] = r3.Vec{X: p.U, Y: p.V}
		if !proofbound.FiniteVec(out.points[i]) {
			return loftCylinderInsetSamples{}, fmt.Errorf("%w: cap inset has a nonfinite station", ErrUnsupported)
		}
	}
	segmentStart := make([]int, len(inset.Outer.Segments))
	for i := range segmentStart {
		segmentStart[i] = -1
	}
	for i, segment := range samples.FaceOf {
		if segment < 0 || segment >= len(segmentStart) {
			return loftCylinderInsetSamples{}, fmt.Errorf("%w: inset walk lost its source segment", ErrUnsupported)
		}
		if segmentStart[segment] < 0 {
			segmentStart[segment] = i
		}
	}
	segment := 0
	for i := range ring {
		if segment >= len(segmentStart) || segmentStart[segment] < 0 {
			return loftCylinderInsetSamples{}, fmt.Errorf("%w: inset line lost its source edge", ErrUnsupported)
		}
		if _, ok := inset.Outer.Segments[segment].(lineSeg); !ok {
			return loftCylinderInsetSamples{}, fmt.Errorf("%w: inset source edge is not a line", ErrUnsupported)
		}
		out.lineStart[i] = segmentStart[segment]
		segment++
		if segment < len(segmentStart) {
			if _, ok := inset.Outer.Segments[segment].(arcSeg); ok {
				if segmentStart[segment] < 0 {
					return loftCylinderInsetSamples{}, fmt.Errorf("%w: inset arc lost its source corner", ErrUnsupported)
				}
				out.arcStart[(i+1)%len(ring)] = segmentStart[segment]
				segment++
			}
		}
	}
	if segment != len(segmentStart) {
		return loftCylinderInsetSamples{}, fmt.Errorf("%w: inset has unmatched offset pieces", ErrUnsupported)
	}
	return out, nil
}
