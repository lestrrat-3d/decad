package planarpose

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarband"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
)

// This file admits exact planar solids to the general pair relation of
// docs/multibody-dynamics-design.md §9.1 and caches each body's §9.2
// convexity certificate. internal/pair decides the relation over the exact
// snapshots built here, and internal/planarband publishes the contact report
// from it; this file owns admission, the snapshots and the certificate cache.
// A body whose held mesh is exact but stands for a true boundary up to a
// positive displacement δ is admitted too; internal/planarband applies
// §10.4's band rule to the held relation.

// heldMeshChord is the chord tolerance an all-planar held-mesh body is
// tessellated at. Nothing on an all-planar body is chorded, so the value only
// keys the body's tessellation cache.
const heldMeshChord = 1.0

// ClassifyExactPair decides the pair through the exact planar kernel
// when both bodies are admitted. It reports false, leaving report untouched,
// when either body is not.
func ClassifyExactPair(ctx context.Context, mesher verifiedmesh.Mesher, report *reportvocab.ContactReport) (bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return false, err
	}
	chord := report.Request.HeldChord.Base()
	a, deltaA, okA, err := SolidAtPose(ctx, budget, mesher, report.A, report.PoseA, chord)
	if err != nil || !okA {
		return false, err
	}
	b, deltaB, okB, err := SolidAtPose(ctx, budget, mesher, report.B, report.PoseB, chord)
	if err != nil || !okB {
		return false, err
	}
	hint := hintOf(report.A, report.B)
	result, err := planar.ClassifyPlanarHinted(&a, &b, hint, budget.Step)
	if err != nil {
		return false, err
	}
	if result.Nearest != (planar.PlanarHint{}) && result.Nearest != hint {
		report.A.PlanarHints.Store(report.B, result.Nearest)
	}
	if err := planarband.Publish(ctx, budget, report, result, &a, &b, deltaA, deltaB, Convexity{Chord: chord, Mesher: mesher}); err != nil {
		return false, err
	}
	return true, budget.Err()
}

// hintOf returns the nearest candidate pair the last exact planar
// relation of a, as the first solid, against b found, or the zero hint. A
// hint never changes a relation (planar.ClassifyPlanarHinted), so one left by a
// pose far from this one, or by a snapshot at another chord, only costs time.
func hintOf(a, b *topo.Body) planar.PlanarHint {
	stored, _ := a.PlanarHints.Load(b)
	hint, _ := stored.(planar.PlanarHint)
	return hint
}

// Convexity reads §9.2 certificates for internal/planarband: Chord is the chord,
// in millimetres, a held mesh with a curved face is read at, and Mesher
// supplies the held mesh.
type Convexity struct {
	Chord  float64
	Mesher verifiedmesh.Mesher
}

// Convex returns the body's cached §9.2 certificate, computing it at the
// identity pose on first use. A canceled computation caches nothing. A
// positive-displacement body's certificate is its held mesh's, read at the
// chord when that mesh has a curved face. It stays in the root package
// because its snapshot (snapshotOf) dispatches on the root payload
// types and tessellates the body.
func (c Convexity) Convex(ctx context.Context, budget *proofbound.WorkBudget, b *topo.Body) (bool, error) {
	bits := math.Float64bits(c.Chord)
	if entry := b.PlanarConvexity.Load(); entry != nil && (entry.ChordFree || entry.ChordBits == bits) {
		return entry.Convex, nil
	}
	snapshot, err := snapshotOf(ctx, budget, c.Mesher, b, c.Chord)
	if err != nil || !snapshot.OK {
		return false, err
	}
	solid, ok, err := planarsnapshot.Place(budget, snapshot, r3.Identity())
	if err != nil || !ok {
		return false, err
	}
	convex, err := planar.PlanarConvex(&solid, budget.Step)
	if err != nil {
		return false, err
	}
	b.PlanarConvexity.Store(&planarsnapshot.ConvexityEntry{ChordBits: bits, ChordFree: snapshot.ChordFree, Convex: convex})
	return convex, nil
}

// SolidAtPose builds the exact held boundary of an admitted planar
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
// (heldMeshSolid), a body with a curved face at chord. The
// returned displacement is δ at the query pose: the body-frame figure times
// an upper bound on the pose's stretch (planarsnapshot.PoseScale). It is zero for an
// exact body.
//
// The snapshot at the identity pose is built once per body and chord and
// cached on the body with its triangles' combinatorial audit
// (snapshotOf); each call maps its vertices through the pose and audits
// the coordinate half (placePlanarSnapshot), so the cache changes no outcome.
func SolidAtPose(ctx context.Context, budget *proofbound.WorkBudget, mesher verifiedmesh.Mesher, b *topo.Body,
	pose r3.Transform, chord float64) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if b == nil || !b.Solid || b.Kind != topo.BodySolid || !planarsnapshot.PositiveAffine(pose) {
		return planar.PlanarSolid{}, none, false, nil
	}
	snapshot, err := snapshotOf(ctx, budget, mesher, b, chord)
	if err != nil || !snapshot.OK {
		return planar.PlanarSolid{}, none, false, err
	}
	solid, ok, err := planarsnapshot.Place(budget, snapshot, pose)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	delta := snapshot.Delta
	if delta.Sign() > 0 {
		delta = proofarith.DyMul(delta, planarsnapshot.PoseScale(pose))
	}
	return solid, delta, true, nil
}

// snapshotOf returns the body's exact snapshot at the identity query
// pose for chord, building and caching it on a miss. An error (cancellation,
// an exhausted budget) caches nothing; a refusal is cached like a snapshot,
// since it reads nothing but the body and the chord.
func snapshotOf(ctx context.Context, budget *proofbound.WorkBudget, mesher verifiedmesh.Mesher, b *topo.Body, chord float64) (*planarsnapshot.SnapshotEntry, error) {
	bits := math.Float64bits(chord)
	if entry := b.PlanarSnapshot.Load(); entry != nil && entry.ServesChord(bits) {
		return entry, nil
	}
	entry := &planarsnapshot.SnapshotEntry{ChordBits: bits, ChordFree: true, Delta: proofarith.DyZero()}
	var err error
	switch payload := b.Payload.(type) {
	case prismrecord.Payload:
		entry.Solid, entry.OK, err = planarband.PrismSolid(ctx, budget, payload, prismFaceIndex(b))
	case facetedrecord.Payload:
		entry.Solid, entry.Delta, entry.OK, err = planarband.FacetedSolid(budget, payload)
	case stitchrecord.Payload:
		entry.Solid, entry.Delta, entry.OK, err = stitchSolid(budget, b, payload)
	case coilshell.Payload:
		entry.Solid, entry.Delta, entry.OK, err = coilSolid(ctx, budget, b, payload)
	default:
		entry.Solid, entry.Delta, entry.OK, entry.ChordFree, err = heldMeshSolid(ctx, budget, mesher, b, chord)
	}
	if err != nil {
		return nil, err
	}
	if entry.OK {
		entry.Topology, entry.OK, err = planar.NewPlanarTopology(len(entry.Solid.Verts), entry.Solid.Tris, budget.Step)
		if err != nil {
			return nil, err
		}
	}
	if !entry.OK {
		entry.Solid, entry.Delta = planar.PlanarSolid{}, proofarith.DyZero()
	}
	b.PlanarSnapshot.Store(entry)
	return entry, nil
}

// prismFaceIndex maps a prism face's provenance role to its index in the
// body's Faces order, the roles evalPrism gives its caps and side walls.
func prismFaceIndex(b *topo.Body) map[string]int {
	faces := topobuild.Faces(b)
	faceRoles := make([][]string, len(faces))
	for i, face := range faces {
		for _, origin := range face.Origins {
			faceRoles[i] = append(faceRoles[i], origin.Role)
		}
	}
	return selectorquery.RoleIndex(faceRoles)
}

// stitchSolid reads a closed all-planar stitched solid off its own
// audited triangle set (§9). Stitch records Tris, TriFaces and VertBound only
// for an all-planar weld, and AuditClean only when the crossing audit ran and
// passed, so a curved or mixed stitch (nil Tris) and a failed audit are not
// admitted; SolidAtPose has already refused an open sheet, which is
// not a solid. The vertices are the welded table's floats at the stitch's
// placement, TriFaces names each triangle's live face, and a face missing
// from topobuild.Faces(b) leaves the snapshot's face map unset.
//
// The displacement is the largest per-vertex weld bound, and at least the
// placement rounding Delta. Each VertBound entry is the bound vertexForClass
// stamps on its live Vertex, the weld class bound with Delta already added,
// and every held triangle point is a convex combination of its corners, so
// no true boundary point lies farther than that bound from the held mesh. It
// is zero for a weld built at the identity whose every class is zero-bound,
// which is then an exact §9 body.
func stitchSolid(budget *proofbound.WorkBudget, b *topo.Body, sp stitchrecord.Payload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if !sp.AuditClean || len(sp.Tris) == 0 || len(sp.TriFaces) != len(sp.Tris) ||
		len(sp.VertBound) != len(sp.Verts) || !measurement.FiniteValues(sp.Delta) || sp.Delta < 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	displacement := sp.Delta
	for _, bound := range sp.VertBound {
		if !measurement.FiniteValues(bound) || bound < 0 {
			return planar.PlanarSolid{}, none, false, nil
		}
		displacement = max(displacement, bound)
	}
	faces := topobuild.Faces(b)
	faceAt := make(map[*topo.Face]int, len(faces))
	for i, face := range faces {
		faceAt[face] = i
	}
	faceOf := make([]int, len(sp.Tris))
	for t, face := range sp.TriFaces {
		at, ok := faceAt[face]
		if !ok {
			faceOf = nil
			break
		}
		faceOf[t] = at
	}
	solid, ok, err := planarsnapshot.HeldSolid(budget, sp.Verts, sp.Tris, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(displacement), true, nil
}

// coilSolid reads a coil off its held shell (docs/helix-design.md Table
// CD row CD5). The shell is a closed all-triangle restatement of the body
// whose every vertex lies within its own β of the true boundary, so its
// displacement is the payload's delta, the same bound its mesh publishes. The
// restatement does not depend on a chord, so coilshell.Tessellate is asked at delta
// itself, the finest tolerance it serves, and the snapshot serves every
// chord. The wall's Faceted face is not planar, which is why this path does
// not go through heldMeshSolid: that arm refuses a curved body at the
// zero chord the planar arm reads.
func coilSolid(ctx context.Context, budget *proofbound.WorkBudget, b *topo.Body,
	cp coilshell.Payload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if !measurement.FiniteValues(cp.Delta) || cp.Delta < 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	mesh, err := coilshell.Tessellate(ctx, b, cp, cp.Delta, false)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return planar.PlanarSolid{}, none, false, ctxErr
		}
		return planar.PlanarSolid{}, none, false, nil
	}
	faces := topobuild.Faces(b)
	faceAt := make(map[*topo.Face]int, len(faces))
	for i, face := range faces {
		faceAt[face] = i
	}
	faceOf := make([]int, len(mesh.Triangles))
	for t, face := range mesh.Source {
		at, ok := faceAt[face]
		if !ok {
			return planar.PlanarSolid{}, none, false, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarsnapshot.HeldSolid(budget, mesh.Vertices, mesh.Triangles, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(cp.Delta), true, nil
}

// heldMeshSolid reads a solid payload without an exact contact family
// of its own (a cap-loop chamfer, a cup, a loft, a sweep, a general revolve)
// off its VerifyAll tessellation, the body's own held mesh at its placement,
// and returns that mesh's Bound as its displacement δ, the two-sided
// displacement between the mesh and the true boundary
// (docs/multibody-dynamics-design.md §10.4). A body whose every face is
// planar is read at heldMeshChord, which chords nothing, and its snapshot
// serves every chord (chordFree); a body with a curved face is read at chord,
// and a zero chord admits none. A source sphere or cylinder keeps its exact
// path and is not admitted here, nor is a body the tessellator refuses or
// whose mesh names a face the body does not carry.
func heldMeshSolid(ctx context.Context, budget *proofbound.WorkBudget, mesher verifiedmesh.Mesher, b *topo.Body,
	chord float64) (planar.PlanarSolid, proofarith.Dyadic, bool, bool, error) {
	none := proofarith.DyZero()
	if _, ok := contactround.SphereRecord(b); ok {
		return planar.PlanarSolid{}, none, false, true, nil
	}
	if _, ok := contactround.CylinderAtPose(b, r3.Identity()); ok {
		return planar.PlanarSolid{}, none, false, true, nil
	}
	faces := topobuild.Faces(b)
	faceAt := make(map[*topo.Face]int, len(faces))
	tolerance, chordFree := heldMeshChord, true
	for i, face := range faces {
		if face.Surface.Kind() != surfacegeom.KindPlane && !face.HeldPlanar {
			tolerance, chordFree = chord, false
		}
		faceAt[face] = i
	}
	if !chordFree && tolerance <= 0 {
		return planar.PlanarSolid{}, none, false, false, nil
	}
	mesh, err := mesher.VerifyAll(ctx, b, tolerance)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return planar.PlanarSolid{}, none, false, chordFree, ctxErr
		}
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return planar.PlanarSolid{}, none, false, chordFree, err
		}
		return planar.PlanarSolid{}, none, false, chordFree, nil
	}
	if !mesh.BoundaryOK || !measurement.FiniteValues(mesh.Bound) || mesh.Bound < 0 ||
		len(mesh.Source) != len(mesh.Triangles) {
		return planar.PlanarSolid{}, none, false, chordFree, nil
	}
	faceOf := make([]int, len(mesh.Triangles))
	for t, face := range mesh.Source {
		at, ok := faceAt[face]
		if !ok {
			return planar.PlanarSolid{}, none, false, chordFree, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarsnapshot.HeldSolid(budget, mesh.Vertices, mesh.Triangles, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, chordFree, err
	}
	return solid, proofarith.MustDyOf(mesh.Bound), true, chordFree, nil
}
