package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file admits exact planar solids to the general pair relation of
// docs/multibody-dynamics-design.md §9.1 and caches each body's §9.2
// convexity certificate. internal/pair decides the relation over the exact
// snapshots built here; this file owns admission and the public report. A
// body whose held mesh is exact but stands for a true boundary up to a
// positive displacement δ is admitted too, and §10.4's band rule turns the
// held relation into the published one.

// planarHeldChord is the chord tolerance an all-planar held-mesh body is
// tessellated at. Nothing on an all-planar body is chorded, so the value only
// keys the body's tessellation cache.
const planarHeldChord = 1.0

// heldChordOf is the chord, in millimetres, a held mesh with a curved face is
// read at under req: the caller's HeldChord, zero when unset, which admits no
// curved body (docs/multibody-dynamics-design.md §10.4). Every reader of the
// chord goes through here, so the chord's source is chosen in one place.
func heldChordOf(req ContactRequest) float64 {
	return req.HeldChord.Base()
}

// planarConvexityEntry is a body's cached convexity certificate. It is read
// at the identity pose: every admitted pose is an affine map with a positive
// determinant, which preserves every orientation sign the certificate reads.
// It is keyed as the snapshot it was read off (planarSnapshotEntry).
type planarConvexityEntry struct {
	chordBits uint64
	chordFree bool
	convex    bool
}

// planarSnapshotEntry is a body's cached exact held snapshot at the identity
// query pose, the body's own placement applied, or its refusal (ok false).
// A snapshot that does not read the held chord (every exact family, and a
// held mesh whose every face is planar) is chordFree and serves every chord;
// a curved held mesh's serves only the chord whose bits it records. Its
// slices are shared by every reader and never written after it is stored.
// topology is the triangle set's combinatorial audit and derived data, which
// no pose changes (pair.PlanarTopology); a snapshot it refuses is a refusal.
type planarSnapshotEntry struct {
	chordBits uint64
	chordFree bool
	ok        bool
	solid     pair.PlanarSolid
	delta     proofarith.Dyadic
	topology  *pair.PlanarTopology
}

// servesChord reports whether the entry stands for the snapshot at chord.
func (e *planarSnapshotEntry) servesChord(bits uint64) bool {
	return e.chordFree || e.chordBits == bits
}

// classifyExactPlanarPair decides the pair through the exact planar kernel
// when both bodies are admitted. It reports false, leaving report untouched,
// when either body is not.
func classifyExactPlanarPair(ctx context.Context, report *ContactReport) (bool, error) {
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return false, err
	}
	chord := heldChordOf(report.Request)
	a, deltaA, okA, err := planarSolidAtPose(ctx, budget, report.A, report.PoseA, chord)
	if err != nil || !okA {
		return false, err
	}
	b, deltaB, okB, err := planarSolidAtPose(ctx, budget, report.B, report.PoseB, chord)
	if err != nil || !okB {
		return false, err
	}
	hint := planarHintOf(report.A, report.B)
	result, err := pair.ClassifyPlanarHinted(&a, &b, hint, budget.step)
	if err != nil {
		return false, err
	}
	if result.Nearest != (pair.PlanarHint{}) && result.Nearest != hint {
		report.A.planarHints.Store(report.B, result.Nearest)
	}
	report.Relation, report.Gap, report.Overlap, report.Manifold = ContactUndecided, nil, nil, nil
	report.Reason = sourceBoxReason(result.Reason)
	if deltaA.Sign() > 0 || deltaB.Sign() > 0 {
		band := planarBandPair{a: &a, b: &b, deltaA: deltaA, deltaB: deltaB}
		if err := band.classify(ctx, budget, report, result); err != nil {
			return false, err
		}
		return true, budget.err()
	}
	band := supportBandOf(report.Request)
	switch result.Relation {
	case pair.Separated:
		report.Relation = ContactSeparated
		gap := sourceBoxScalar(*result.Gap)
		report.Gap = &gap
		if err := planarSupportBand(budget, report, &a, &b, result, band); err != nil {
			return false, err
		}
	case pair.Touching, pair.Overlapping:
		report.Relation = ContactOverlapping
		if result.Relation == pair.Touching {
			report.Relation = ContactTouching
			gap := Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}
			report.Gap = &gap
		}
		// §9.3 and §9.6 need one convex side for a manifold; without one an
		// overlap's reason names the missing certificate, and a touch takes
		// §10.5's non-convex guest path inside publishPlanarManifold.
		convexA, err := planarConvexity(ctx, budget, report.A, chord)
		if err != nil {
			return false, err
		}
		convexB, err := planarConvexity(ctx, budget, report.B, chord)
		if err != nil {
			return false, err
		}
		if !convexA && !convexB && result.Relation == pair.Overlapping {
			report.Reason = ContactNonConvex
			break
		}
		report.Reason = ContactNoNormalProof
		if err := publishPlanarManifold(budget, report, &a, &b, result, convexA, convexB, band); err != nil {
			return false, err
		}
	default:
		if report.Reason == ContactNoReason {
			report.Reason = ContactAmbiguousFeature
		}
	}
	return true, budget.err()
}

// planarHintOf returns the nearest candidate pair the last exact planar
// relation of a, as the first solid, against b found, or the zero hint. A
// hint never changes a relation (pair.ClassifyPlanarHinted), so one left by a
// pose far from this one, or by a snapshot at another chord, only costs time.
func planarHintOf(a, b *Body) pair.PlanarHint {
	stored, _ := a.planarHints.Load(b)
	hint, _ := stored.(pair.PlanarHint)
	return hint
}

// planarConvexity returns the body's cached §9.2 certificate, computing it at
// the identity pose on first use. A canceled computation caches nothing. A
// positive-displacement body's certificate is its held mesh's, read at chord
// when that mesh has a curved face.
func planarConvexity(ctx context.Context, budget *workBudget, b *Body, chord float64) (bool, error) {
	bits := math.Float64bits(chord)
	if entry := b.planarConvexity.Load(); entry != nil && (entry.chordFree || entry.chordBits == bits) {
		return entry.convex, nil
	}
	snapshot, err := planarSnapshotOf(ctx, budget, b, chord)
	if err != nil || !snapshot.ok {
		return false, err
	}
	solid, ok, err := placePlanarSnapshot(budget, snapshot, r3.Identity())
	if err != nil || !ok {
		return false, err
	}
	convex, err := pair.PlanarConvex(&solid, budget.step)
	if err != nil {
		return false, err
	}
	b.planarConvexity.Store(&planarConvexityEntry{chordBits: bits, chordFree: snapshot.chordFree, convex: convex})
	return convex, nil
}

// planarSolidAtPose builds the exact held boundary of an admitted planar
// solid under a query pose (§9): a prism whose section is all whole LineSeg
// edges with zero deltas, a zero-bound faceted Boolean, either directly or
// through its saved exact mesh and a translation-only placement, or a closed
// all-planar stitched solid welded at the identity with every vertex bound
// zero. Every vertex is the exact dyadic image of recorded coordinates under
// the recorded placement and the pose; nothing is read from a rounded
// transient body.
//
// §10.4 admits held meshes whose true boundary lies within a positive
// two-sided displacement δ of them: a positive-bound faceted Boolean, read
// off its payload with δ its mesh bound, a placed or certificate-welded
// stitched solid, read off its triangle set with δ its largest vertex bound,
// and every other solid payload without an exact contact family of its own,
// read off its VerifyAll tessellation with δ that mesh's Bound
// (planarHeldMeshSolid), a body with a curved face at chord. The
// returned displacement is δ at the query pose: the body-frame figure times
// an upper bound on the pose's stretch (planarPoseScale). It is zero for an
// exact body.
//
// The snapshot at the identity pose is built once per body and chord and
// cached on the body with its triangles' combinatorial audit
// (planarSnapshotOf); each call maps its vertices through the pose and audits
// the coordinate half (placePlanarSnapshot), so the cache changes no outcome.
func planarSolidAtPose(ctx context.Context, budget *workBudget, b *Body,
	pose r3.Transform, chord float64) (pair.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if b == nil || !b.solid || b.kind != BodySolid || !positiveAffine(pose) {
		return pair.PlanarSolid{}, none, false, nil
	}
	snapshot, err := planarSnapshotOf(ctx, budget, b, chord)
	if err != nil || !snapshot.ok {
		return pair.PlanarSolid{}, none, false, err
	}
	solid, ok, err := placePlanarSnapshot(budget, snapshot, pose)
	if err != nil || !ok {
		return pair.PlanarSolid{}, none, false, err
	}
	delta := snapshot.delta
	if delta.Sign() > 0 {
		delta = proofarith.DyMul(delta, planarPoseScale(pose))
	}
	return solid, delta, true, nil
}

// placePlanarSnapshot maps a cached snapshot's vertices through pose into a
// fresh vertex slice and audits the result: the snapshot's topology already
// passed the combinatorial half of pair.CheckPlanarSolid, and
// pair.CheckPlanarPose runs the coordinate half and attaches the pose's
// derived data. The triangle and face slices are shared with the cache,
// clipped so no append can reach its storage.
func placePlanarSnapshot(budget *workBudget, snapshot *planarSnapshotEntry,
	pose r3.Transform) (pair.PlanarSolid, bool, error) {
	solid := pair.PlanarSolid{Verts: make([]proofarith.DyV3, len(snapshot.solid.Verts)),
		Tris: slices.Clip(snapshot.solid.Tris), Faces: slices.Clip(snapshot.solid.Faces)}
	place := newExactContactMap(pose)
	for i, v := range snapshot.solid.Verts {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		solid.Verts[i] = place.apply(v)
	}
	audited, err := pair.CheckPlanarPose(&solid, snapshot.topology, budget.step)
	if err != nil || !audited {
		return pair.PlanarSolid{}, false, err
	}
	return solid, true, nil
}

// planarSnapshotOf returns the body's exact snapshot at the identity query
// pose for chord, building and caching it on a miss. An error (cancellation,
// an exhausted budget) caches nothing; a refusal is cached like a snapshot,
// since it reads nothing but the body and the chord.
func planarSnapshotOf(ctx context.Context, budget *workBudget, b *Body, chord float64) (*planarSnapshotEntry, error) {
	bits := math.Float64bits(chord)
	if entry := b.planarSnapshot.Load(); entry != nil && entry.servesChord(bits) {
		return entry, nil
	}
	entry := &planarSnapshotEntry{chordBits: bits, chordFree: true, delta: proofarith.DyZero()}
	var err error
	switch payload := b.payload.(type) {
	case prismPayload:
		entry.solid, entry.ok, err = planarPrismSolid(ctx, budget, payload, prismFaceIndex(b))
	case facetedPayload:
		entry.solid, entry.delta, entry.ok, err = planarFacetedSolid(budget, payload)
	case stitchPayload:
		entry.solid, entry.delta, entry.ok, err = planarStitchSolid(budget, b, payload)
	default:
		entry.solid, entry.delta, entry.ok, entry.chordFree, err = planarHeldMeshSolid(ctx, budget, b, chord)
	}
	if err != nil {
		return nil, err
	}
	if entry.ok {
		entry.topology, entry.ok, err = pair.NewPlanarTopology(len(entry.solid.Verts), entry.solid.Tris, budget.step)
		if err != nil {
			return nil, err
		}
	}
	if !entry.ok {
		entry.solid, entry.delta = pair.PlanarSolid{}, proofarith.DyZero()
	}
	b.planarSnapshot.Store(entry)
	return entry, nil
}

// planarPoseScale is an exact upper bound, at least one, on how far the
// linear part L of an admitted transform stretches any length. The largest
// absolute row sum g of the Gram matrix LᵀL bounds its largest eigenvalue,
// which is |L|², and (1 + g)/2 bounds √g from above. A valid r3 transform is
// orthonormal to rounding, so the bound sits a hair above one; it is charged
// all the same, since the held mesh and its displacement move through the
// exact float map, not through a rotation.
func planarPoseScale(t r3.Transform) proofarith.Dyadic {
	basis := t.Basis()
	columns := [3]proofarith.DyV3{proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ)}
	one := proofarith.DyInt(1)
	g := one
	for i := range 3 {
		row := proofarith.DyZero()
		for j := range 3 {
			row = proofarith.DyAdd(row, proofarith.DyAbs(proofarith.DvDot(columns[i], columns[j])))
		}
		if proofarith.DyCmp(row, g) > 0 {
			g = row
		}
	}
	return proofarith.DyShift(proofarith.DyAdd(one, g), -1)
}

// positiveAffine admits a finite transform whose exact basis determinant is
// positive. Such a map preserves every orientation sign, so a snapshot's
// outward winding and convexity survive it.
func positiveAffine(t r3.Transform) bool {
	if !t.IsValid() || !finiteVec(t.Translation()) {
		return false
	}
	basis := t.Basis()
	if !finiteVec(basis.EX) || !finiteVec(basis.EY) || !finiteVec(basis.EZ) {
		return false
	}
	ex, ey, ez := proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ)
	return proofarith.DvDot(ex, proofarith.DvCross(ey, ez)).Sign() > 0
}

// prismFaceIndex maps a prism face's provenance role to its index in the
// body's Faces order, the roles evalPrism gives its caps and side walls.
func prismFaceIndex(b *Body) map[string]int {
	index := make(map[string]int)
	for i, face := range b.Faces() {
		for _, origin := range face.Origins() {
			index[origin.Role] = i
		}
	}
	return index
}

// planarPrismSolid lifts an all-LineSeg section through its frame and sweep
// levels. The caps reuse the tessellator's triangulation, which this function
// then proves: every cap triangle is exactly counterclockwise and, through the
// closed-mesh audit, the caps' boundary chain is the section loops, so the
// triangles tile the section exactly. Each triangle records its face from
// the roles in faceIndex: the start cap at z0, the end cap at z1, and
// side(loop, segment) for a wall; a missing role leaves Faces unset.
func planarPrismSolid(ctx context.Context, budget *workBudget,
	pp prismPayload, faceIndex map[string]int) (pair.PlanarSolid, bool, error) {
	if pp.surfaceResult || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!finiteMeasurementValues(pp.z0, pp.z1) || pp.z0 >= pp.z1 || !positiveAffine(pp.xform) ||
		!finiteVec(pp.frame.Origin()) || !finiteVec(pp.frame.U()) ||
		!finiteVec(pp.frame.V()) || !finiteVec(pp.frame.N()) {
		return pair.PlanarSolid{}, false, nil
	}
	fu, fv, fn := proofarith.DyVec(pp.frame.U()), proofarith.DyVec(pp.frame.V()), proofarith.DyVec(pp.frame.N())
	if proofarith.DvDot(fu, proofarith.DvCross(fv, fn)).Sign() <= 0 {
		return pair.PlanarSolid{}, false, nil
	}
	var pts []Point2
	loops := make([][]int, 0, 1+len(pp.profile.Holes))
	for role, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		indices, ok := planarLoopPoints(loop, &pts)
		if !ok {
			return pair.PlanarSolid{}, false, nil
		}
		area := planarLoopArea(pts, indices)
		if (role == 0 && area.Sign() <= 0) || (role > 0 && area.Sign() >= 0) {
			return pair.PlanarSolid{}, false, nil
		}
		loops = append(loops, indices)
	}
	caps, err := triangulate2DContext(ctx, pts, loops)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return pair.PlanarSolid{}, false, ctxErr
		}
		return pair.PlanarSolid{}, false, nil
	}
	n := len(pts)
	solid := pair.PlanarSolid{Verts: make([]proofarith.DyV3, 2*n)}
	origin := proofarith.DyVec(pp.frame.Origin())
	z := [2]proofarith.Dyadic{proofarith.MustDyOf(pp.z0), proofarith.MustDyOf(pp.z1)}
	for i, p := range pts {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		local := proofarith.DvAdd(origin, proofarith.DvAdd(dyScaleVec(fu, proofarith.MustDyOf(p.U)),
			dyScaleVec(fv, proofarith.MustDyOf(p.V))))
		for level := range z {
			solid.Verts[level*n+i] = exactContactTransform(pp.xform,
				proofarith.DvAdd(local, dyScaleVec(fn, z[level])))
		}
	}
	for _, tri := range caps {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		if planarCross2(pts[tri[0]], pts[tri[1]], pts[tri[2]]).Sign() <= 0 {
			return pair.PlanarSolid{}, false, nil
		}
		solid.Tris = append(solid.Tris, [3]int{tri[0], tri[2], tri[1]},
			[3]int{n + tri[0], n + tri[1], n + tri[2]})
	}
	start, okStart := faceIndex[roleCapStart]
	end, okEnd := faceIndex[roleCapEnd]
	mapped := okStart && okEnd
	for range caps {
		solid.Faces = append(solid.Faces, start, end)
	}
	for li, loop := range loops {
		for i, from := range loop {
			to := loop[(i+1)%len(loop)]
			solid.Tris = append(solid.Tris, [3]int{from, to, n + to}, [3]int{from, n + to, n + from})
			side, ok := faceIndex[fmt.Sprintf("side(%d,%d)", li, i)]
			mapped = mapped && ok
			solid.Faces = append(solid.Faces, side, side)
		}
	}
	if !mapped {
		solid.Faces = nil
	}
	return solid, true, nil
}

// planarLoopPoints appends one loop's walk-start points to pts and returns
// their indices. It admits only whole LineSeg edges that chain exactly.
func planarLoopPoints(loop LoopRecord, pts *[]Point2) ([]int, bool) {
	if len(loop.Segments) < 3 {
		return nil, false
	}
	indices := make([]int, 0, len(loop.Segments))
	var first, last Point2
	for i, segment := range loop.Segments {
		line, ok := segment.(LineSeg)
		if !ok {
			return nil, false
		}
		start, end := line.Start, line.End
		switch {
		case line.TStart == 0 && line.TEnd == 1:
		case line.TStart == 1 && line.TEnd == 0:
			start, end = end, start
		default:
			return nil, false
		}
		if !finiteMeasurementValues(start.U, start.V, end.U, end.V) || start == end ||
			(i > 0 && start != last) {
			return nil, false
		}
		if i == 0 {
			first = start
		}
		last = end
		indices = append(indices, len(*pts))
		*pts = append(*pts, start)
	}
	if last != first {
		return nil, false
	}
	return indices, true
}

// planarLoopArea is twice the exact signed area of a loop.
func planarLoopArea(pts []Point2, loop []int) proofarith.Dyadic {
	area := proofarith.DyZero()
	for i, from := range loop {
		a, b := pts[from], pts[loop[(i+1)%len(loop)]]
		area = proofarith.DyAdd(area, proofarith.DySubScalar(
			proofarith.DyMul(proofarith.MustDyOf(a.U), proofarith.MustDyOf(b.V)),
			proofarith.DyMul(proofarith.MustDyOf(b.U), proofarith.MustDyOf(a.V))))
	}
	return area
}

// planarCross2 is the exact (b−a)×(c−a) of three section points.
func planarCross2(a, b, c Point2) proofarith.Dyadic {
	au, av := proofarith.MustDyOf(a.U), proofarith.MustDyOf(a.V)
	bu := proofarith.DySubScalar(proofarith.MustDyOf(b.U), au)
	bv := proofarith.DySubScalar(proofarith.MustDyOf(b.V), av)
	cu := proofarith.DySubScalar(proofarith.MustDyOf(c.U), au)
	cv := proofarith.DySubScalar(proofarith.MustDyOf(c.V), av)
	return proofarith.DySubScalar(proofarith.DyMul(bu, cv), proofarith.DyMul(bv, cu))
}

// planarFacetedSolid reads a Boolean's held mesh. A zero-bound mesh is its
// exact boundary. A translation-only placement reads the saved exact source
// mesh moved by the exact placement translation, after a reject-only check
// that every rebuilt vertex lies within the published bound of it. Any other
// positive-bound mesh is read as held, with its mesh bound returned as the
// displacement δ of §10.4.
func planarFacetedSolid(budget *workBudget, pp facetedPayload) (pair.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if len(pp.verts) == 0 || len(pp.tris) == 0 {
		return pair.PlanarSolid{}, none, false, nil
	}
	if pp.meshBound != 0 || pp.volSymDiff != 0 {
		if !finiteMeasurementValues(pp.meshBound) || pp.meshBound < 0 {
			return pair.PlanarSolid{}, none, false, nil
		}
		solid, ok, err := planarFacetedSourceSolid(budget, pp)
		if err != nil || ok {
			return solid, none, ok, err
		}
		if pp.meshBound == 0 {
			return pair.PlanarSolid{}, none, false, nil
		}
	}
	solid, ok, err := planarHeldSolid(budget, pp.verts, pp.tris, pp.faceOf)
	if err != nil || !ok {
		return pair.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(pp.meshBound), true, nil
}

// planarFacetedSourceSolid is the saved exact source mesh of a zero-bound
// Boolean moved by a translation-only placement.
func planarFacetedSourceSolid(budget *workBudget, pp facetedPayload) (pair.PlanarSolid, bool, error) {
	if !facetedTranslationOnly(pp.xform) || len(pp.exactSourceVerts) != len(pp.verts) ||
		len(pp.exactSourceTris) != len(pp.tris) {
		return pair.PlanarSolid{}, false, nil
	}
	for i, tri := range pp.tris {
		if tri != pp.exactSourceTris[i] {
			return pair.PlanarSolid{}, false, nil
		}
	}
	solid, ok, err := planarHeldSolid(budget, pp.exactSourceVerts, pp.tris, pp.faceOf)
	if err != nil || !ok {
		return pair.PlanarSolid{}, false, err
	}
	bound := proofarith.MustDyOf(pp.meshBound)
	for i := range solid.Verts {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		if !finiteVec(pp.verts[i]) {
			return pair.PlanarSolid{}, false, nil
		}
		solid.Verts[i] = exactContactTransform(pp.xform, solid.Verts[i])
		difference := proofarith.DvSub(solid.Verts[i], proofarith.DyVec(pp.verts[i]))
		if proofarith.DyCmp(proofarith.DvDot(difference, difference), proofarith.DyMul(bound, bound)) > 0 {
			return pair.PlanarSolid{}, false, nil
		}
	}
	return solid, true, nil
}

// planarHeldSolid lifts a held triangle mesh to an exact snapshot. faceOf
// names each triangle's face; a length mismatch leaves Faces unset.
func planarHeldSolid(budget *workBudget, verts []r3.Vec, tris [][3]int, faceOf []int) (pair.PlanarSolid, bool, error) {
	solid := pair.PlanarSolid{Verts: make([]proofarith.DyV3, len(verts)),
		Tris: append([][3]int(nil), tris...)}
	if len(faceOf) == len(tris) {
		solid.Faces = append([]int(nil), faceOf...)
	}
	for i, v := range verts {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		if !finiteVec(v) {
			return pair.PlanarSolid{}, false, nil
		}
		solid.Verts[i] = proofarith.DyVec(v)
	}
	return solid, true, nil
}

// planarStitchSolid reads a closed all-planar stitched solid off its own
// audited triangle set (§9). Stitch records tris, triFaces and vertBound only
// for an all-planar weld, and auditClean only when the crossing audit ran and
// passed, so a curved or mixed stitch (nil tris) and a failed audit are not
// admitted; planarSolidAtPose has already refused an open sheet, which is
// not a solid. The vertices are the welded table's floats at the stitch's
// placement, triFaces names each triangle's live face, and a face missing
// from b.Faces() leaves the snapshot's face map unset.
//
// The displacement is the largest per-vertex weld bound, and at least the
// placement rounding delta. Each vertBound entry is the bound vertexForClass
// stamps on its live Vertex, the weld class bound with delta already added,
// and every held triangle point is a convex combination of its corners, so
// no true boundary point lies farther than that bound from the held mesh. It
// is zero for a weld built at the identity whose every class is zero-bound,
// which is then an exact §9 body.
func planarStitchSolid(budget *workBudget, b *Body, sp stitchPayload) (pair.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if !sp.auditClean || len(sp.tris) == 0 || len(sp.triFaces) != len(sp.tris) ||
		len(sp.vertBound) != len(sp.verts) || !finiteMeasurementValues(sp.delta) || sp.delta < 0 {
		return pair.PlanarSolid{}, none, false, nil
	}
	displacement := sp.delta
	for _, bound := range sp.vertBound {
		if !finiteMeasurementValues(bound) || bound < 0 {
			return pair.PlanarSolid{}, none, false, nil
		}
		displacement = max(displacement, bound)
	}
	faces := b.Faces()
	faceAt := make(map[*Face]int, len(faces))
	for i, face := range faces {
		faceAt[face] = i
	}
	faceOf := make([]int, len(sp.tris))
	for t, face := range sp.triFaces {
		at, ok := faceAt[face]
		if !ok {
			faceOf = nil
			break
		}
		faceOf[t] = at
	}
	solid, ok, err := planarHeldSolid(budget, sp.verts, sp.tris, faceOf)
	if err != nil || !ok {
		return pair.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(displacement), true, nil
}

// planarHeldMeshSolid reads a solid payload without an exact contact family
// of its own (a cap-loop chamfer, a cup, a loft, a sweep, a general revolve)
// off its VerifyAll tessellation, the body's own held mesh at its placement,
// and returns that mesh's Bound as its displacement δ, the two-sided
// displacement between the mesh and the true boundary
// (docs/multibody-dynamics-design.md §10.4). A body whose every face is
// planar is read at planarHeldChord, which chords nothing, and its snapshot
// serves every chord (chordFree); a body with a curved face is read at chord,
// and a zero chord admits none. A source sphere or cylinder keeps its exact
// path and is not admitted here, nor is a body the tessellator refuses or
// whose mesh names a face the body does not carry.
func planarHeldMeshSolid(ctx context.Context, budget *workBudget, b *Body,
	chord float64) (pair.PlanarSolid, proofarith.Dyadic, bool, bool, error) {
	none := proofarith.DyZero()
	if _, ok := sourceSphereRecord(b); ok {
		return pair.PlanarSolid{}, none, false, true, nil
	}
	if _, ok := sourceCylinderAtPose(b, r3.Identity()); ok {
		return pair.PlanarSolid{}, none, false, true, nil
	}
	faces := b.Faces()
	faceAt := make(map[*Face]int, len(faces))
	tolerance, chordFree := planarHeldChord, true
	for i, face := range faces {
		if !face.isPlanar() {
			tolerance, chordFree = chord, false
		}
		faceAt[face] = i
	}
	if !chordFree && tolerance <= 0 {
		return pair.PlanarSolid{}, none, false, false, nil
	}
	mesh, err := tessellateContext(ctx, b, units.Millimeters(tolerance), VerifyAll)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return pair.PlanarSolid{}, none, false, chordFree, ctxErr
		}
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return pair.PlanarSolid{}, none, false, chordFree, err
		}
		return pair.PlanarSolid{}, none, false, chordFree, nil
	}
	if !mesh.boundaryOK || !finiteMeasurementValues(mesh.bound) || mesh.bound < 0 ||
		len(mesh.source) != len(mesh.triangles) {
		return pair.PlanarSolid{}, none, false, chordFree, nil
	}
	faceOf := make([]int, len(mesh.triangles))
	for t, face := range mesh.source {
		at, ok := faceAt[face]
		if !ok {
			return pair.PlanarSolid{}, none, false, chordFree, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarHeldSolid(budget, mesh.vertices, mesh.triangles, faceOf)
	if err != nil || !ok {
		return pair.PlanarSolid{}, none, false, chordFree, err
	}
	return solid, proofarith.MustDyOf(mesh.bound), true, chordFree, nil
}

// planarBandPair is one admitted pair at least one of whose held meshes
// carries a positive displacement, with both displacements at the query
// poses (docs/multibody-dynamics-design.md §10.4).
type planarBandPair struct {
	a, b           *pair.PlanarSolid
	deltaA, deltaB proofarith.Dyadic
}

// classify turns the exact held relation into §10.4's published one. With δ
// the summed displacement, every true boundary point lies within δ of the
// held pair's: a held gap within the lifted band b = max(SupportBand, δ) over
// which the zero-δ body hosts a lifted set is a band carrying that set
// (liftedBand); a held gap whose lower end exceeds δ is a true gap with δ
// charged; a held touch, or a held gap or penetration depth at most δ, puts
// the true signed separation in [−2δ, 2δ], ContactBand; a held vertex deeper
// than δ inside the other held body is a true overlap (pair.PlanarDeepVertex)
// carrying the held penetration patch charged with δ (overlapManifold).
// Anything between is undecided.
func (p *planarBandPair) classify(ctx context.Context, budget *workBudget, report *ContactReport,
	result pair.PlanarResult) error {
	delta := proofarith.DyAdd(p.deltaA, p.deltaB).Rat()
	switch result.Relation {
	case pair.Separated:
		value, bound := proofarith.FloatRat(result.Gap.ValueMM), proofarith.FloatRat(result.Gap.BoundMM)
		upper := new(big.Rat).Add(value, bound)
		if banded, err := p.liftedBand(budget, report, result, upper); err != nil || banded {
			return err
		}
		charged := new(big.Rat).Add(bound, delta)
		if published := ratFloatUp(charged); finiteMeasurementValues(published) &&
			value.Cmp(proofarith.FloatRat(published)) > 0 {
			report.Relation, report.Reason = ContactSeparated, ContactNoReason
			report.Gap = &Measurement{Value: units.Millimeters(result.Gap.ValueMM),
				Bound: units.Millimeters(published), Exactness: exactnessFromBound(published)}
			return nil
		}
		if upper.Cmp(delta) <= 0 {
			return p.publishBand(ctx, budget, report, nil)
		}
		report.Reason = ContactNoGapProof
	case pair.Touching:
		convexA, convexB, err := p.convexity(ctx, budget, report)
		if err != nil {
			return err
		}
		if !convexA && !convexB {
			if err := p.publishBand(ctx, budget, report, nil); err != nil {
				return err
			}
			report.Reason = ContactNonConvex
			return nil
		}
		manifold, err := pair.PlanarTouchManifold(p.a, p.b, result.Contacts, convexA, convexB, budget.step)
		if err != nil {
			return err
		}
		if err := p.publishBand(ctx, budget, report, manifold.Points); err != nil {
			return err
		}
		if manifold.Points == nil {
			report.Reason = sourceBoxReason(manifold.Reason)
		}
	case pair.Overlapping:
		deep, err := pair.PlanarDeepVertex(p.a, p.b, proofarith.DyAdd(p.deltaA, p.deltaB), budget.step)
		if err != nil {
			return err
		}
		if deep {
			report.Relation, report.Reason = ContactOverlapping, ContactNoNormalProof
			return p.overlapManifold(ctx, budget, report, result)
		}
		report.Reason = ContactNoGapProof
		convexA, convexB, err := p.convexity(ctx, budget, report)
		if err != nil || !convexA || !convexB {
			return err
		}
		points, err := pair.PlanarPenetrationManifold(p.a, p.b, budget.step)
		if err != nil || points == nil {
			return err
		}
		for _, point := range points {
			// The held depth is minus the separation's lower end.
			depth := new(big.Rat).Sub(proofarith.FloatRat(point.Separation.BoundMM),
				proofarith.FloatRat(point.Separation.ValueMM))
			if depth.Cmp(delta) > 0 {
				return nil
			}
		}
		return p.publishBand(ctx, budget, report, points)
	}
	return nil
}

// liftedBand publishes a held separated pair whose gap's upper end is within
// the lifted band b = max(SupportBand, δ) as ContactBand when some face of a
// zero-δ body hosts a nonempty lifted set within b (§10.4, §10.5). A support
// plane hosted by a displaced body publishes no lifted set: its held face
// only approximates the true face's direction. The band reads δ from below,
// so a resting displaced pair, placed with its held gap at most δ, keeps its
// lifted set however small SupportBand is. The gap becomes [0 ± (g + δ)], g
// the held gap's upper float, and the manifold is the lifted sets charged
// with δ (chargedManifold). It reports whether it published.
func (p *planarBandPair) liftedBand(budget *workBudget, report *ContactReport, result pair.PlanarResult,
	upper *big.Rat) (bool, error) {
	band := p.liftedWidth(report.Request)
	hostA, hostB := p.deltaA.Sign() == 0, p.deltaB.Sign() == 0
	if upper.Cmp(band.Rat()) > 0 || !hostA && !hostB {
		return false, nil
	}
	lifted, err := planarLiftedSet(budget, p.a, p.b, result, planarSupportPlanes(p.a, p.b, hostA, hostB), band,
		false)
	if err != nil || len(lifted) == 0 {
		return false, err
	}
	g := ratFloatUp(upper)
	charged := ratFloatUp(new(big.Rat).Add(proofarith.FloatRat(g), proofarith.DyAdd(p.deltaA, p.deltaB).Rat()))
	if !finiteMeasurementValues(g, charged) {
		return false, nil
	}
	report.Relation, report.Reason = ContactBand, ContactNoReason
	report.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(charged),
		Exactness: exactnessFromBound(charged)}
	manifold, reason, err := p.chargedManifold(budget, report, lifted, nil)
	if err != nil {
		return false, err
	}
	report.Manifold, report.Reason = manifold, reason
	return true, nil
}

// liftedWidth is the lifted band b = max(SupportBand, δ) of a displaced pair.
func (p *planarBandPair) liftedWidth(req ContactRequest) proofarith.Dyadic {
	band, delta := supportBandOf(req), proofarith.DyAdd(p.deltaA, p.deltaB)
	if proofarith.DyCmp(band, delta) < 0 {
		return delta
	}
	return band
}

// overlapManifold publishes the held penetration patch of a pair proven to
// overlap (§10.4): §9.3's convex-convex patch or §9.6's face-local one, each
// body read as M grown by its own δ, then the support plane's lifted set
// within the lifted band, every point charged with δ (chargedManifold). A
// patch the kernel withholds keeps the reason it names, or the one report
// carries.
func (p *planarBandPair) overlapManifold(ctx context.Context, budget *workBudget, report *ContactReport,
	result pair.PlanarResult) error {
	convexA, convexB, err := p.convexity(ctx, budget, report)
	if err != nil {
		return err
	}
	if !convexA && !convexB {
		report.Reason = ContactNonConvex
		return nil
	}
	points, planes, reason, err := planarOverlapPatch(budget, p.a, p.b, result, convexA, convexB, p.deltaA, p.deltaB)
	if err != nil {
		return err
	}
	if points == nil {
		if reason != pair.NoReason {
			report.Reason = sourceBoxReason(reason)
		}
		return nil
	}
	manifold, contactReason, err := p.chargedManifold(budget, report, points, nil)
	if err != nil || manifold == nil {
		report.Reason = contactReason
		return err
	}
	lifted, err := planarLiftedSet(budget, p.a, p.b, result, planes, p.liftedWidth(report.Request), true)
	if err != nil {
		return err
	}
	if lifted != nil {
		extra, contactReason, err := p.chargedManifold(budget, report, lifted, nil)
		if err != nil || extra == nil {
			report.Reason = contactReason
			return err
		}
		manifold.Points = append(manifold.Points, extra.Points...)
	}
	report.Manifold, report.Reason = manifold, ContactNoReason
	return nil
}

func (p *planarBandPair) convexity(ctx context.Context, budget *workBudget,
	report *ContactReport) (bool, bool, error) {
	chord := heldChordOf(report.Request)
	convexA, err := planarConvexity(ctx, budget, report.A, chord)
	if err != nil {
		return false, false, err
	}
	convexB, err := planarConvexity(ctx, budget, report.B, chord)
	return convexA, convexB, err
}

// publishBand publishes ContactBand with Gap [−2δ, 2δ] and, when points is
// not nil, the held manifold charged with the band, every Separation the
// band itself (chargedManifold).
func (p *planarBandPair) publishBand(ctx context.Context, budget *workBudget, report *ContactReport,
	points []pair.PatchPoint) error {
	band := ratFloatUp(new(big.Rat).Mul(big.NewRat(2, 1), proofarith.DyAdd(p.deltaA, p.deltaB).Rat()))
	if !finiteMeasurementValues(band) {
		report.Reason = ContactNoGapProof
		return nil
	}
	report.Relation, report.Reason = ContactBand, ContactNoNormalProof
	gap := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(band), Exactness: exactnessFromBound(band)}
	report.Gap = &gap
	if points == nil {
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	manifold, reason, err := p.chargedManifold(budget, report, points, &gap)
	if err != nil {
		return err
	}
	report.Manifold, report.Reason = manifold, reason
	return nil
}

// chargedManifold charges a held manifold with the displacements, as §9.4 and
// §10.4 state for a positive-displacement body: the points are the held
// pair's, each witness ball grows by its own body's δ, since a held vertex
// or foot is a point of the held mesh and Bound does not place it on the true
// surface, and each Separation is band when band is not nil and otherwise the
// held one widened by the summed δ. The normal must be the exact face normal
// of a body with no displacement, read at a face of that body that holds the
// point: a held face of a displaced body only approximates the true face's
// direction, so a point whose normal no exact face supplies withholds the
// manifold with ContactNoNormalProof.
func (p *planarBandPair) chargedManifold(budget *workBudget, report *ContactReport, points []pair.PatchPoint,
	band *Measurement) (*ContactManifold, ContactReason, error) {
	exact := -1
	switch {
	case p.deltaA.Sign() == 0:
		exact = 0
	case p.deltaB.Sign() == 0:
		exact = 1
	}
	if exact < 0 {
		return nil, ContactNoNormalProof, nil
	}
	solids := [2]*pair.PlanarSolid{p.a, p.b}
	for _, point := range points {
		if err := budget.step(); err != nil {
			return nil, ContactNoReason, err
		}
		feature := point.A
		if exact == 1 {
			feature = point.B
		}
		normal, ok := planarFaceNormal(solids[exact], feature)
		if !ok || !proofarith.DvIsZero(proofarith.DvCross(normal, point.Normal)) {
			return nil, ContactNoNormalProof, nil
		}
	}
	features, err := newPlanarFeatureMap(report.A, report.B, p.a, p.b)
	if err != nil {
		return nil, ContactNoReason, err
	}
	manifold, reason := planarPatchManifold(report.Request, points, features)
	if manifold == nil {
		return nil, reason, nil
	}
	resolution := proofarith.FloatRat(report.Request.PointResolution.Base())
	delta := proofarith.DyAdd(p.deltaA, p.deltaB).Rat()
	for i := range manifold.Points {
		point := &manifold.Points[i]
		for side, position := range []*VecMeasurement{&point.OnA, &point.OnB} {
			own := p.deltaA
			if side == 1 {
				own = p.deltaB
			}
			published := ratFloatUp(new(big.Rat).Add(proofarith.FloatRat(position.Bound.Base()), own.Rat()))
			if !finiteMeasurementValues(published) || proofarith.FloatRat(published).Cmp(resolution) > 0 {
				return nil, ContactPointTooCoarse, nil
			}
			position.Bound, position.Exactness = units.Millimeters(published), exactnessFromBound(published)
		}
		if band != nil {
			point.Separation = *band
			continue
		}
		widened := ratFloatUp(new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), delta))
		if !finiteMeasurementValues(widened) {
			return nil, ContactNoGapProof, nil
		}
		point.Separation.Bound, point.Separation.Exactness = units.Millimeters(widened), exactnessFromBound(widened)
	}
	return manifold, ContactNoReason, nil
}

// planarFaceNormal is the exact outward normal of the one face a facet
// feature names, read off the first triangle that face owns.
func planarFaceNormal(solid *pair.PlanarSolid, feature pair.PatchFeature) (proofarith.DyV3, bool) {
	if feature.Kind != pair.FeatureFacet || len(feature.Faces) != 1 || len(solid.Faces) != len(solid.Tris) {
		return proofarith.DyV3{}, false
	}
	for t, tri := range solid.Tris {
		if solid.Faces[t] != feature.Faces[0] {
			continue
		}
		a := solid.Verts[tri[0]]
		return proofarith.DvCross(proofarith.DvSub(solid.Verts[tri[1]], a), proofarith.DvSub(solid.Verts[tri[2]], a)), true
	}
	return proofarith.DyV3{}, false
}
