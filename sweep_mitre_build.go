package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds docs/sweep-design.md §16's mitred polyline sweep: the
// §16.3 construction over exact rationals, Table SM rows SM5 through SM8, the
// single rounding into the held vertex table, Table BM's topology, and
// §16.6's four readings, all taken over the rational vertices.
//
// Every section is the image of the one before it under its span's wall
// lines, and every wall line of one span passes through that span's apex (or
// is parallel to the span when its ratio is one). So every wall quad is
// exactly planar, and the map from one section to the next is a projective
// map on a convex region holding the whole section (SM6 states each of its
// conditions as a linear inequality over the vertex, so it holds on the hull
// once it holds at every vertex). That map carries a simple polygon with
// holes to a simple polygon with holes and carries a triangulation of one to
// a triangulation of the other, which is why capEnd reuses capStart's
// triangulation index for index rather than triangulating a second time.

// mitredSection is one loop-major section polygon: vertex v of loop i sits at
// index loopIdx[i][j].
type mitredSection []sweepRatVec

// mitredConstruction is §16.3's exact record before placement: every section
// polygon, the path points and the cap triangulation of P_0.
type mitredConstruction struct {
	sections []mitredSection
	loopIdx  [][]int
	capTris  [][3]int
	anchor   sweepRatVec
}

// constructMitredSweep triangulates the recorded profile once, then adapts
// its spans to §16.3's exact section construction.
func constructMitredSweep(ctx context.Context, mp mitredSweepPayload) (mitredConstruction, error) {
	pts2, loopIdx, err := mitredSweepLoops(mp.profile)
	if err != nil {
		return mitredConstruction{}, err
	}
	capTris, err := triangulation.Triangulate(ctx, pts2, loopIdx)
	if err != nil {
		return mitredConstruction{}, wrapLoftTriangulationError(err)
	}
	spans := make([]sweepmitre.Span, len(mp.path.records))
	for k, record := range mp.path.records {
		spans[k] = sweepmitre.Span{Start: record.start, End: record.end}
	}
	built, err := sweepmitre.Construct(ctx, mp.plane, pts2, loopIdx, spans, mp.factors)
	if err != nil {
		return mitredConstruction{}, err
	}
	sections := make([]mitredSection, len(built.Sections))
	for k, section := range built.Sections {
		sections[k] = mitredSection(section)
	}
	return mitredConstruction{sections: sections, loopIdx: loopIdx, capTris: capTris, anchor: built.Anchor}, nil
}

// mitredSpanLengthLower is λ_j of §16.3: the lower endpoint of the certified
// enclosure of the span's length, independent of platform sqrt or FMA.
func mitredSpanLengthLower(start, end r3.Vec) (*big.Rat, error) {
	return sweepmitre.SpanLengthLower(start, end)
}

// mitredRequireWalls enforces SM7 over one span.
func mitredRequireWalls(k int, loopIdx [][]int, from, to mitredSection) error {
	return sweepmitre.RequireWalls(k, loopIdx, from, to)
}

// mitredPlace applies the accumulated placement to a rational point exactly.
func mitredPlace(xform r3.Transform, p sweepRatVec) sweepRatVec {
	return sweepmitre.Place(xform, p)
}

// mitredRound rounds one rational coordinate to its nearest float and
// returns the exact gap.
func mitredRound(r *big.Rat) (float64, *big.Rat, bool) {
	return sweepmitre.Round(r)
}

// evalMitredSweep builds the body for one payload record: §16.3's exact
// construction and placement, the single rounding, the triangle set and
// orientation, SM8's crossing audit, Table BM's topology and §16.6's
// readings. The first build and every placement run it.
func evalMitredSweep(ctx context.Context, d *Document, ref producerID, mp mitredSweepPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	c, err := constructMitredSweep(ctx, mp)
	if err != nil {
		return nil, err
	}

	stride := len(c.sections[0])
	exact := make([]sweepRatVec, 0, stride*len(c.sections))
	var localExact []sweepRatVec
	if mp.localVol6 == nil && mp.xform != r3.Identity() {
		localExact = make([]sweepRatVec, 0, cap(exact))
	}
	for _, section := range c.sections {
		for _, p := range section {
			if localExact != nil {
				localExact = append(localExact, p)
			}
			exact = append(exact, mitredPlace(mp.xform, p))
		}
	}
	anchor := mitredPlace(mp.xform, c.anchor)

	// Rounding, once: every vertex to its nearest float per coordinate, with
	// that vertex's own 3D gap read exactly from its largest coordinate gap and
	// rounded up (docs/faceted-vertex-bounds-design.md §2.1), and delta the
	// largest of them.
	verts := make([]r3.Vec, len(exact))
	vertexBound := make([]float64, len(exact))
	delta := 0.0
	for v, p := range exact {
		var coords [3]float64
		worst := new(big.Rat)
		for axis := range 3 {
			f, gap, ok := mitredRound(p[axis])
			if !ok {
				return nil, fmt.Errorf(`%w: a mitred sweep vertex runs past the representable float64 range`, ErrUnsupported)
			}
			coords[axis] = f
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
		verts[v] = r3.NewVec(coords[0], coords[1], coords[2])
		if worst.Sign() != 0 {
			w, _ := worst.Float64()
			vertexBound[v] = proofbound.Radius3D(proofbound.ProvenUpRound(w))
			delta = max(delta, vertexBound[v])
		}
	}

	a := assembleMitredSweep(c, stride)
	localVol6, localMoments := mp.localVol6, mp.localMoments
	if localVol6 == nil {
		if localExact == nil {
			localExact = exact
		}
		localVol6, localMoments = mitredVolumeMoments(localExact, a.tris, c.anchor)
	}
	vol6, moments := sweepmitre.PlacedVolumeMoments(localVol6, localMoments, mp.xform)
	switch vol6.Sign() {
	case 0:
		return nil, fmt.Errorf(`%w: the mitred sweep encloses no volume`, ErrDegenerate)
	case -1:
		// §16.3's orientation rule: one exact sign for the whole shell.
		a.reversed = true
		for t, tri := range a.tris {
			a.tris[t] = [3]int{tri[0], tri[2], tri[1]}
		}
		vol6.Neg(vol6)
		for axis := range moments {
			moments[axis].Neg(moments[axis])
		}
	}
	for t, tri := range a.tris {
		if loftmesh.TriangleCollapsed(verts, tri) {
			return nil, fmt.Errorf(`%w: rounding the mitred sweep's vertices collapsed its triangle %d (%s)`, ErrUnsupported, t, a.roles[a.triFace[t]])
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, a.tris); err != nil {
		var contact *loftmesh.LoftContactError
		if errors.As(err, &contact) {
			return nil, fmt.Errorf(`%w: mitred sweep faces %s and %s %s`, ErrDegenerate,
				a.roles[a.triFace[contact.I]], a.roles[a.triFace[contact.J]], contact.Reason)
		}
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	areas, err := mitredTriangleAreas(ctx, exact, a.tris)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	faces, err := buildMitredTopology(ctx, body, ref, a, exact, verts, areas, delta)
	if err != nil {
		return nil, err
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	body.volume = mitredVolume(vol6)
	if body.centroid, err = mitredCentroid(anchor, vol6, moments); err != nil {
		return nil, err
	}
	body.bounds = mitredBounds(exact)
	areaLo, areaHi := new(big.Rat), new(big.Rat)
	for _, enclosure := range areas {
		areaLo.Add(areaLo, enclosure[0])
		areaHi.Add(areaHi, enclosure[1])
	}
	value, bound := mitredEnclosure(areaLo, areaHi)
	body.area = Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.SquareMillimeters(bound),
	}
	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}
	if !finiteMeasurementValues(bound) {
		return nil, fmt.Errorf(`%w: the mitred sweep's area has no finite bound`, ErrUnsupported)
	}

	mp.exact, mp.verts, mp.tris, mp.triFace, mp.faceRoles, mp.delta = exact, verts, a.tris, a.triFace, a.roles, delta
	mp.vertexBound = vertexBound
	mp.localVol6, mp.localMoments = localVol6, localMoments
	body.payload = mp
	return body, nil
}
