package decad

import (
	"context"
	"fmt"
	"math"
	"strings"
	"sync"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the tessellation half of the export slice (core §11,
// docs/evaluator-design.md §9): per-surface analytic tessellators over the
// evaluator's own prism and cup payloads, with a proven per-facet deviation
// bound and facets that remember their source face. A boolean-faceted body
// restates its held mesh here; the exact-predicate mesh boolean that produces
// it lives in boolean_mesh.go.

// Mesh is a triangle mesh: an OUTPUT of [Body.Tessellate], never the body
// representation (core §3 invariant #1 — the public vocabulary stays
// Body → Face → Edge → Vertex). Vertices lie on the evaluator's held boundary;
// curve chording and inherited payload displacement are bounded by Bound. A Mesh
// is immutable: the accessors return copies of its slices.
type Mesh struct {
	_ noCopy
	// n is the node data internal packages read and build (internal/topo).
	n topo.Mesh
}

// sourceBound reads docs/tessellation-design.md §2's sourceBound(face) for one
// of the mesh's own source faces. A face the record does not carry is an
// evaluator invariant failure — the caller's own sentinel decides how loud —
// and NEVER a zero, which would claim the face is held exactly.
func (m *Mesh) sourceBound(f *topo.Face) (float64, bool) {
	d, ok := m.n.FaceBound[f]
	return d, ok
}

// setVertexBounds publishes a per-vertex record (docs/faceted-vertex-bounds-
// design.md §2.1) and, from it, every source face's bound as the largest
// corner entry over that face's own triangles, lifting bound to match. The
// caller's record carries exactly one entry per vertex, and source is
// already set; the mesh keeps its own copy.
func (m *Mesh) setVertexBounds(beta []float64) {
	m.n.VertexBound = append([]float64(nil), beta...)
	for t, tri := range m.n.Triangles {
		topobuild.SetFaceBound(&m.n, m.n.Source[t], max(beta[tri[0]], beta[tri[1]], beta[tri[2]]))
	}
}

// vertexBounds reads β(v) for every vertex (docs/faceted-vertex-bounds-
// design.md §2.1): the mesh's own record when its payload publishes one, and
// otherwise the largest faceBound over the faces whose facets touch v. The
// derived reading is a claim the mesh already made — v lies on a facet whose
// face's true patch is within that face's bound of it — and taking the
// maximum over every incident face keeps each facet's own displacement,
// chord sagitta included, as its corner maximum. A source face missing from
// faceBound is a broken evaluator (ErrBooleanFailed, as facesOfMesh reports
// it), never a zero. The returned slice is the caller's own.
func (m *Mesh) vertexBounds() ([]float64, error) {
	if m.n.VertexBound != nil {
		return append([]float64(nil), m.n.VertexBound...), nil
	}
	beta := make([]float64, len(m.n.Vertices))
	for t, tri := range m.n.Triangles {
		d, ok := m.sourceBound(m.n.Source[t])
		if !ok {
			return nil, fmt.Errorf(`%w: a mesh facet's source face states no displacement bound`, ErrBooleanFailed)
		}
		for _, v := range tri {
			beta[v] = max(beta[v], d)
		}
	}
	return beta, nil
}

// Vertices returns the mesh vertex positions in millimetres (core §5.2).
// Every vertex lies on the held boundary. Its deviation from the denoted body,
// including inherited payload displacement, is covered by Bound.
func (m *Mesh) Vertices() []r3.Vec { return append([]r3.Vec(nil), m.n.Vertices...) }

// Triangles returns the facets as index triples into Vertices, wound
// counter-clockwise seen from outside the body. On a [BodySheet] there is no
// "outside": the winding is counter-clockwise seen from the shell's own
// positive side (docs/surface-design.md §2.3) — the same side [Face.NormalAt]
// answers on for a wall the solid the sheet came from would have carried. The
// indices describe the mesh's own connectivity; they are not selectors
// (core §3 invariant #3).
//
// "Outside the body" presupposes an EMBEDDED surface, which is what the
// facet-contact audit proves, so the word holds as a geometric statement only
// when [Mesh.BoundaryVerified] is true. On a mesh built below [VerifyBoundary]
// the winding is still consistent across every shared edge and still outward
// under the construction's own convention — the signed-volume orientation
// audit runs at every level — but a self-intersecting closed mesh can carry a
// positive signed volume while having points where "outside" names nothing.
// For a renderer culling back faces or lighting them the two readings agree;
// for a caller who needs the guarantee, ask for it.
func (m *Mesh) Triangles() [][3]int { return append([][3]int(nil), m.n.Triangles...) }

// SourceFaces returns, parallel to Triangles, the analytic face each facet
// approximates — the provenance the mesh boolean groups its output by
// (docs/evaluator-design.md §9). The elements are the body's live faces.
func (m *Mesh) SourceFaces() []*Face { return faceHandles(m.n.Source) }

// Bound returns the proven deviation bound: no point of the body's boundary
// lies farther than this from the mesh, and vice versa. It is the largest
// complete displacement the tessellation used, including curve chording and
// inherited payload coordinate bounds. Its chording component is at most the
// requested tolerance — it is a PROVEN upper bound on the chord-to-arc
// deviation rather than that deviation itself, so it can read above the
// deviation a chord actually takes by the factor [Body.Tessellate] states;
// inherited payload displacement is added on top, so
// Bound can exceed that tolerance. It is zero only when every held boundary
// coordinate is exact. A scalar quantity is a units.Value (core §5.1): Kind
// Length, millimetres.
func (m *Mesh) Bound() units.Value { return units.Millimeters(m.n.Bound) }

// Tessellate approximates the body's boundary as a triangle mesh whose
// chording deviates from the held analytic faces by no more than tol. Inherited
// payload displacement is added to the returned Bound. It is an OUTPUT, not
// the representation (core §11). tol is a magnitude: Kind Length
// ([ErrUnitKind] otherwise), finite ([ErrNotFinite]), non-negative
// ([ErrNegativeMagnitude]); a zero tolerance asks for a chord that is the
// curve and is [ErrDegenerate].
//
// Planar faces triangulate exactly. Circular and free-form boundaries are
// chorded at parameter samples chosen once per boundary curve and shared by
// every face that meets it — a cap and the cylinder wall use the same chording
// of their shared edge — so the mesh is watertight and consistently oriented by
// construction, and Bound carries the complete displacement actually taken. A
// free-form (Tier A NURBS) boundary is chorded by exact dyadic bisection of its
// own Bézier chain, measured against docs/spline-design.md §6.2.1's sagitta,
// and refuses rather than coarsen when its exact arithmetic passes the fixed
// free-form work budget or when the chain would carry more chords than one
// curve may.
//
// Two things follow from the chording sagitta being PROVEN rather than tight
// (docs/tessellation-design.md §3 derives it; tessellation.ChordSagitta implements it).
// First, the chording component of [Mesh.Bound] sits above the deviation the
// chords take by the factor (x/sin x)² at x = a chord's own quarter angle —
// at most π²/9, about 9.66% high, at the coarsest chording a closed walk
// reaches (three chords over a full circle), and 0.83% at ten. Second, the
// chord count is chosen against that same proven figure, so the finest tol
// this mesh admits is set by it and not by the tighter true sagitta: a tol
// within about 3.06e-9 relative of the finest chording the per-curve cap
// allows leaves no admissible count and is [ErrUnsupported]. Both point the
// same way — a finer mesh, or a refusal, never a coarser mesh under a claim
// this package cannot prove.
//
// Prism, revolve, cup, loft, cap-loop chamfer and boolean-built bodies
// tessellate. A one-span [Sweep] reuses its prism or Revolve mesh.
// A composite Sweep with line and circular profile walks chooses shared
// profile stations across its spans and maps each facet to the body's live
// face. A solid composite mesh carries an occupied-volume proof; a sheet
// mesh does not. Composite free-form profile walks and meridian poles are
// [ErrUnsupported] (docs/sweep-design.md Table D row D2).
// A revolved body meshes its meridian section and one GLOBAL angular sequence,
// so every generator face shares its whole latitude edge and a full turn
// closes with no seam; its two coordinate stages — the construction that computes each sample
// and the placement that moves it — are reserved from tol before any chord is
// chosen, and a tol they exhaust is [ErrUnsupported]. What tol then buys is
// split between the meridian and the angles, so a CIRCULAR generator (a sphere
// or a torus wall) is chorded along its meridian as well as around the axis. A
// section a bounded number of refinements cannot prove simple, correctly
// nested and free of non-adjacent facet contact is [ErrUnsupported] rather
// than a mesh, and so is a positive-radius ring that collapses onto itself: a
// revolve mesh is refined or refused, never snapped, welded or coarsened. It
// also proves the volume it and the body it stands for differ by, so [Union],
// [Cut] and [Intersect] take a revolved body as an operand.
//
// A cap-loop chamfer result meshes its trimmed side walls, its chamfer band and
// its two cap faces from ONE chord count per wall walk, shared by the side
// wall's own rings, the band patch ruled off them and the cap contour the band
// ends on, so no strip is sampled at two densities and the mesh is watertight
// by construction. Where every band is a whole turn or joins only line-line
// miters and exactly tangent corners, it also proves the volume it and the
// body it stands for differ by, so [Union], [Cut] and [Intersect] take it as
// an operand; a band with a mitered circular wall or a reflex corner carries
// no such proof, and that mesh serves export while the booleans refuse it.
//
// A tapered extrude (a draft body, docs/draft-design.md) meshes as that same
// band with no straight slab: one ruled wall per section walk between the near
// cap's rim and the far section, a sharp miter at every line-line corner,
// convex or reflex, and both caps triangulated. Where every circular wall
// meets its neighbours exactly tangent, it also proves the volume it and the
// body differ by, so [Union], [Cut] and [Intersect] take it as an operand.
//
// A lofted body RESTATES the flat triangle set its construction already built
// and audited: nothing is chorded here, so tol binds nothing on that path and
// the returned Bound is the payload's own facet departure, which can sit either
// side of tol. A boolean-built body also only RESTATES its held mesh, so tol
// must be at least the body's own Bound: a
// finer tol is [ErrUnsupported], because the analytic identity is gone and no
// finer mesh can be proven. A prism the analytic prism boolean assembled holds
// its section within a proven displacement of the section it denotes
// (docs/prism-boolean-design.md §7); that displacement is reserved from tol
// before any chord is chosen, so a tol it exhausts is [ErrUnsupported] too.
// A body this evaluator did not build at all is also [ErrUnsupported].
//
// A [BodySheet] built by a surface-result prism or revolve
// (`WithSurfaceResult`) meshes its walls exactly as the solid the same
// record would have built. A prism sheet, and a partial-sweep revolve
// sheet, omit both caps: no cap triangulation runs and no cap face appears
// in SourceFaces. A full-turn revolve mints no cap in either kind, so its
// sheet mesh is bit-identical to the solid's and carries no free edge at
// all. The mandatory audit runs docs/tessellation-design.md §1.2's
// manifold-with-boundary check in place of the closed-mesh audit a solid
// takes, over the same directed-edge structure — with no free edge, the two
// audits agree with no arm of their own. The signed-volume orientation
// check runs only when the mesh is closed — a solid, or a full-turn sheet —
// and never on an open partial-sweep sheet, where that sum is
// anchor-dependent and decides nothing. A sheet's areaSlack drops the cap
// terms it no longer carries, and it publishes no occupied-volume proof at
// all — [Union], [Cut] and [Intersect] refuse a sheet operand outright
// (docs/surface-design.md Table X), so the absence costs nothing a caller
// reaches through this method. Export still succeeds: export.STL and
// export.OBJ write a sheet's mesh exactly as they write a solid's.
//
// [WithVerification] chooses how much of the mesh's proof this call runs. The
// default is [VerifyAll]: every audit and proof the body's payload supports,
// which is the only mesh [Union], [Cut] and [Intersect] accept. A lower level
// returns the SAME mesh — same vertices, same indices, same order, same proven
// Bound — and simply declines to prove some of it, which both costs less and
// reaches tolerances the facet-pair audit's own work ceiling refuses. The
// mesh says which proofs it ended up with through [Mesh.BoundaryVerified] and
// [Mesh.VolumeVerified].
//
// It returns ctx.Err() unchanged when ctx is canceled before or during
// tessellation. A nil ctx is [ErrDegenerate] (core §12): the context is polled
// rather than merely stored, so a nil one is a caller mistake this call cannot
// carry out.
func (b *Body) Tessellate(ctx context.Context, tol units.Value, opts ...TessellateOption) (*Mesh, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a tessellation`, ErrDegenerate)
	}
	folded := make([]option.Interface, len(opts))
	for i, o := range opts {
		folded[i] = o
	}
	verify, err := foldVerification(folded, VerifyAll)
	if err != nil {
		return nil, err
	}
	return tessellateContext(ctx, b, tol, verify)
}

type tessellationCountKey struct{}

// tessellationCount is an internal test hook: it counts, per body, the
// tessellations tessellateContext builds under a context carrying it — every
// build, never a cache hit. It is safe for concurrent use.
type tessellationCount struct {
	mu    sync.Mutex
	built map[*Body]int
}

func (c *tessellationCount) note(b *Body) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.built == nil {
		c.built = map[*Body]int{}
	}
	c.built[b]++
}

// of returns how many tessellations of b were built.
func (c *tessellationCount) of(b *Body) int {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.built[b]
}

// tessellateContext is the read-only evaluator's cancellable tessellation
// entry. It builds only an unowned Mesh and never touches document state.
//
// It is also the one writer of the mesh's boundaryOK reading and the one place
// a level below VerifyAll withholds docs/tessellation-design.md §2's area and
// volume proofs. Both live here rather than in each payload path so the
// contract holds for every path by construction: a path that skipped some
// terms of a proof cannot publish the rest of it, and a path that runs no
// facet-contact audit cannot forget to say what it did prove.
func tessellateContext(ctx context.Context, b *Body, tol units.Value, verify Verification) (*Mesh, error) {
	// core §12's rule for every operation that takes a context: a nil one
	// cannot be polled, and every cancellation check below would dereference
	// it, so it is refused at the entry before the body is examined.
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a tessellation`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	chord, err := sectionrecord.MagnitudeIn(tol, units.Length, units.Millimeter, "the chord tolerance")
	if err != nil {
		return nil, err
	}
	if chord == 0 {
		return nil, fmt.Errorf(`%w: a zero chord tolerance admits no chord`, ErrDegenerate)
	}
	if b.n.Payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot tessellate a body it did not build`, ErrUnsupported)
	}
	key := math.Float64bits(chord)
	if cached := b.n.Tess.Load(); cached != nil && cached.ChordBits == key && cached.Verify == int(verify) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return meshHandle(cached.Mesh), nil
	}
	if count, ok := ctx.Value(tessellationCountKey{}).(*tessellationCount); ok {
		count.note(b)
	}
	mesh, err := tessellateBodyContext(ctx, b, chord, verify)
	if err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		mesh.withholdProofs()
	}
	mesh.n.BoundaryOK = verify >= VerifyBoundary || !payloadAuditsFacetContact(b.n.Payload)
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	b.n.Tess.Store(&topo.TessEntry{ChordBits: key, Verify: int(verify), Mesh: meshNode(mesh)})
	return mesh, nil
}

// tessellateBodyContext dispatches one body to its payload's own tessellator.
// verify reaches only the paths that would otherwise COMPUTE a proof the level
// withholds — prism, patch, one-span sweep, cup, revolve, cap-loop chamfer, draft and the revolve-backed
// curved stitch route.
// A restatement path (faceted, loft, all-planar stitch) copies its proof terms
// off the payload at no cost and publishes them unconditionally;
// tessellateContext withholds them afterwards.
func tessellateBodyContext(ctx context.Context, b *Body, chord float64, verify Verification) (*Mesh, error) {
	if p, ok := b.n.Payload.(allEdgeChamferPayload); ok {
		return tessellateAllEdgeChamfer(ctx, b, p, chord, verify)
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok {
		return tessellateFaceted(ctx, b, fp, chord)
	}
	if cp, ok := b.n.Payload.(cupPayload); ok {
		return tessellateCup(ctx, b, cp.View(), chord, verify)
	}
	if sp, ok := b.n.Payload.(stackedPrismPayload); ok {
		return tessellateStacked(ctx, b, sp, chord, verify)
	}
	if bp, ok := b.n.Payload.(brepPayload); ok {
		return tessellateBrep(ctx, b, bp, chord, verify)
	}
	if lp, ok := b.n.Payload.(loftPayload); ok {
		// The loft path exactly restates the payload's complete set for a
		// solid or its recorded wall range for a sheet, with no chording
		// (internal/loftmesh.RestateLoft's own doc comment owns why).
		return tessellateLoft(ctx, b, lp)
	}
	if tp, ok := b.n.Payload.(twistedSweepPayload); ok {
		if chord < tp.Delta {
			return nil, fmt.Errorf(`%w: the twisted sweep's held boundary is %g mm from its true wall; %g mm is finer`, ErrUnsupported, tp.Delta, chord)
		}
		if tp.Mesh != nil {
			mesh := bornMesh(&Mesh{n: *tp.Mesh})
			return mesh, nil
		}
		mesh, err := tessellateLoft(ctx, b, tp.Held, tp.SourceRoles)
		if err != nil {
			return nil, err
		}
		for _, face := range mesh.n.Source {
			topobuild.SetFaceBound(&mesh.n, face, tp.Delta)
		}
		mesh.n.Bound = tp.Delta
		mesh.n.AreaSlack = tp.AreaSlack
		mesh.n.VolSymDiff = tp.VolSymDiff
		mesh.n.SymDiffOK = true
		return mesh, nil
	}
	if tp, ok := b.n.Payload.(compositeTwistedSweepPayload); ok {
		if chord < tp.Delta {
			return nil, fmt.Errorf(`%w: the composite twist's held boundary is %g mm from its true wall; %g mm is finer`,
				ErrUnsupported, tp.Delta, chord)
		}
		mesh := bornMesh(&Mesh{n: *tp.Mesh})
		return mesh, nil
	}
	if pp, ok := b.n.Payload.(patchPayload); ok {
		return tessellatePatch(ctx, b, pp, chord, verify)
	}
	if rp, ok := b.n.Payload.(revolvePayload); ok {
		return tessellateRevolve(ctx, b, rp, chord, verify)
	}
	if cbp, ok := b.n.Payload.(capBlendPayload); ok {
		return tessellateCapBlend(ctx, b, cbp, chord, verify)
	}
	if dp, ok := b.n.Payload.(draftPayload); ok {
		return tessellateDraft(ctx, b, dp, chord, verify)
	}
	if dp, ok := b.n.Payload.(twoSidedDraftPayload); ok {
		return tessellateTwoSidedDraft(ctx, b, dp, chord, verify)
	}
	if sp, ok := b.n.Payload.(stitchPayload); ok {
		if sp.tris == nil {
			return tessellateStitchCurved(ctx, b, sp, chord, verify)
		}
		// The all-planar stitch restatement takes no chord tolerance at all
		// (tessellate_stitch.go's own doc comment owns why), the same
		// reasoning tessellateLoft's own arm above states for a loft.
		return tessellateStitch(ctx, b, sp)
	}
	if pp, ok := b.n.Payload.(bodyPatchPayload); ok {
		return tessellateBodyPatch(ctx, b, pp, chord, verify)
	}
	if cp, ok := b.n.Payload.(chainPayload); ok {
		// The chain-fed ribbon path takes no chord tolerance either, on the
		// identical reasoning: every wall is already an exact planar quad,
		// so there is no chording decision for a tolerance to bind
		// (tessellate_chain.go's own doc comment owns why).
		return tessellateChain(ctx, b, cp)
	}
	if sp, ok := b.n.Payload.(chainSweepPayload); ok {
		if len(sp.Spans) != 0 {
			return tessellateChainSweep(ctx, b, sp, chord, verify)
		}
		if sp.Arc {
			return tessellateChainArcSweep(ctx, b, sp, chord, verify)
		}
		// the chain Sweep's one-span straight reduction builds through the
		// identical chainrecord.Build a plain chain Extrude does, over
		// the chainPayload it wraps unchanged (internal/sweeprecord's
		// FinishChain), so it reuses that same restatement rather
		// than a second one keyed on the wrapper type.
		return tessellateChain(ctx, b, sp.Chain)
	}
	if mp, ok := b.n.Payload.(mitredSweepPayload); ok {
		// docs/sweep-design.md Table DM row DM2: an exact restatement of the
		// held triangles, refused below the payload's own delta.
		return tessellateMitredSweep(ctx, b, mp, chord)
	}
	if cp, ok := b.n.Payload.(coilPayload); ok {
		// docs/helix-design.md Table CD row CD2: an exact restatement of the
		// held triangles, refused below the payload's own delta.
		return tessellateCoil(ctx, b, cp, chord, verify)
	}
	if sp, ok := b.n.Payload.(sweepPayload); ok {
		// One-span sweeps reuse the prism or revolve mesh of their exact
		// analytic reduction. A sheet carries no occupied-volume proof.
		if len(sp.Spans) != 0 {
			return tessellateCompositeSweep(ctx, b, sp, chord, verify)
		}
		if sp.Arc {
			return tessellateRevolveWithRoles(ctx, b, sp.Revolve, chord, verify, sweepArcRole(sp.ReverseArcCaps))
		}
		return tessellatePrism(ctx, b, sp.Prism, sweepWallRole, chord, verify)
	}
	pp, ok := b.n.Payload.(prismPayload)
	if !ok {
		// Chording is per payload kind. Name both the staged kind and the
		// implemented set so the refusal cannot misstate evaluator reach.
		return nil, fmt.Errorf(`%w: tessellation does not support payload %s; supported payload classes are prism, patch, stacked prism, brep, chain-fed prism, sweep, mitred sweep, coil, revolve, cup, loft, cap-loop chamfer, draft (tapered extrude), stitch, and faceted`, ErrUnsupported, reportvocab.PayloadTypeName(b.n.Payload))
	}
	return tessellatePrism(ctx, b, pp, prismWallRole, chord, verify)
}

// prismWallRole names the wall face prismrecord.Build minted for segment seg
// of section loop loop: Table B's side(i,j).
func prismWallRole(loop, seg int) string { return fmt.Sprintf("side(%d,%d)", loop, seg) }

// sweepWallRole names the same wall on a one-span straight sweep, whose
// reduction rewrote every side(i,j) to carry the path-span index ahead of
// them (internal/sweeprecord's prefixSpanZeroRole): side(0,i,j).
func sweepWallRole(loop, seg int) string { return fmt.Sprintf("side(0,%d,%d)", loop, seg) }

// sweepArcRole maps the revolve reduction's source roles to those minted by
// sweeprecord.FinishArc. A reversed axis exchanges only the two cap names.
func sweepArcRole(reverseCaps bool) func(string) string {
	return func(role string) string {
		if suffix, ok := strings.CutPrefix(role, "side("); ok {
			return "side(0," + suffix
		}
		if reverseCaps {
			switch role {
			case provenance.RoleCapStart:
				return provenance.RoleCapEnd
			case provenance.RoleCapEnd:
				return provenance.RoleCapStart
			}
		}
		return role
	}
}

// tessellatePrism chords a prismPayload build. wallRole names the face each
// section walk's wall carries, which is the one thing a prism and the
// one-span straight sweep that reduced to one name differently.
func tessellatePrism(ctx context.Context, b *Body, pp prismPayload, wallRole func(loop, seg int) string, chord float64, verify Verification) (*Mesh, error) {
	return tessellatePrismWithCountFloor(ctx, b, pp, wallRole, chord, verify, nil, 0)
}

func tessellatePrismWithCountFloor(ctx context.Context, b *Body, pp prismPayload,
	wallRole func(loop, seg int) string, chord float64, verify Verification,
	countFloor func(loop, seg int) int, freeformTarget float64) (*Mesh, error) {
	// sheet is docs/surface-design.md §4.1's own flag, read once: a surface
	// result omits both caps from its wall build (internal/prismrecord/build.go), and every
	// arm below that would otherwise touch a cap face, a cap triangulation or
	// a cap's own area/volume term reads it to skip that half of the work
	// rather than fault a role the build never minted.
	sheet := pp.SurfaceResult

	// Every mesh vertex lands on the RECORDED section, which a payload carrying a
	// section displacement holds only within that displacement of the section its
	// construction DENOTES (docs/prism-boolean-design.md §7). The displacement is
	// therefore part of the mesh's deviation before a single chord is chosen, so
	// docs/tessellation-design.md §5 RESERVES it from the chord budget here, and a
	// tolerance that cannot pay for it refuses, the same shape as
	// tessellateFaceted's refusal of a tolerance below a held mesh bound. The
	// reservation keeps chording plus this displacement within tol; it covers no
	// other term, so the per-end axial displacement added at the end of the build
	// can still lift the complete Bound above tol, which §1's Tolerance row
	// allows. Both downward nudges are proven margin:
	// the subtraction's own rounding is at most half an ulp, which the first
	// covers, and the second pays for the upward-rounded sum this bound is
	// published through at the end of the build. Every payload a caller draws
	// carries a zero displacement and chords against the requested tolerance
	// unchanged.
	budget := chord
	if pp.SectionDelta > 0 {
		budget = freeform.DownRound(freeform.DownRound(chord - pp.SectionDelta))
		if budget <= 0 {
			requested := units.Millimeters(chord)
			displacement := units.Millimeters(pp.SectionDelta)
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s; retry with a tolerance greater than %s`, ErrUnsupported, requested, displacement, displacement)
		}
	}

	// Facets remember their source face (docs/evaluator-design.md §9); the
	// provenance roles are how the payload's walks name the faces evalPrism
	// built from them.
	byRole := map[string]*topo.Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = faceNode(f)
		}
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	// A sheet's build never attaches capStart/capEnd to the shell
	// (internal/prismrecord/build.go), so byRole carries no such role for one: leave both
	// nil and skip every cap-only step below rather than fault a role this
	// evaluator's own reach never minted.
	var capStart, capEnd *topo.Face
	if !sheet {
		var err error
		capStart, err = faceOfRole(provenance.RoleCapStart)
		if err != nil {
			return nil, err
		}
		capEnd, err = faceOfRole(provenance.RoleCapEnd)
		if err != nil {
			return nil, err
		}
	}

	// One boundary polyline per loop: sample j is walk j's own start (the
	// junction shared with the previous walk) plus, for a circular walk, its
	// interior chord samples. Each 2D sample owns one bottom and one top mesh
	// vertex, so every face that meets a boundary curve reuses the SAME
	// chording — watertightness by construction.
	mesh := bornMesh(new(Mesh))
	var chorded []tessellation.PrismLoop[*topo.Face]
	// One free-form counter for the whole chorded record (see tessellation.ChordLoop).
	work := freeform.NewFreeformWork()
	if freeformTarget > 0 {
		work.RaiseLimit(sweeprecord.CompositeFitWorkLimit)
	}
	// The build that produced this body already resolved every boundary
	// segment's walk and published the set onto the payload (internal/prismrecord/build.go,
	// docs/evaluator-design.md §8). A tessellation is one of the passes
	// docs/spline-design.md §5.2 names as free to REPLAY that recorded charge
	// instead of doing the work again: chording reads the walks back and
	// charges this fresh counter what resolving them cost, so the ceiling binds
	// the chording exactly as resolving would have. A payload carrying no
	// resolution of THIS record — a boolean result, a modify op's rewritten
	// section, a body an older evaluator built — leaves pw nil and every loop
	// resolves through walkOf as before.
	pw := pp.Walks
	if pw.Reusable(pp.Profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		pw = nil
	}
	loops := append([]loopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var floor func(survey2d.SideWalk) int
		if countFloor != nil {
			floor = func(w survey2d.SideWalk) int { return countFloor(li, w.Segs[0]) }
		}
		cl, err := tessellation.ChordLoopWithCountFloor(ctx, loop, budget, pp.Z1-pp.Z0, work, pw, li, func(w survey2d.SideWalk) (*topo.Face, error) {
			return faceOfRole(wallRole(li, w.Segs[0]))
		}, stationbound.ChordStationBound, floor, freeformTarget)
		if err != nil {
			return nil, err
		}
		chorded = append(chorded, tessellation.PrismLoop[*topo.Face]{Samples: cl.Samples, FaceOf: cl.FaceOf,
			SagOf: cl.SagOf, BoundOf: cl.BoundOf, MaxSag: cl.MaxSag,
			WallSlack: cl.WallSlack, CapSlack: cl.CapSlack, SegmentArea: cl.SegmentArea,
			Walks: cl.Walks, PerimeterUpper: cl.PerimeterUpper})
	}
	topology := tessellation.PrismWalls(chorded, sheet, pp.AxialDelta())
	mesh.n.AreaSlack = topology.AreaSlack
	pts2, sampleBound := topology.Points, topology.Bounds
	faceTrim, faceAxial := topology.FaceTrim, topology.FaceAxial
	walks, perimeterUpper, segmentArea := topology.Walks, topology.PerimeterUpper, topology.SegmentArea

	// The mesh vertices: bottom and top of every boundary sample, placed
	// through the payload — exactly on the analytic boundary.
	mesh.n.Vertices = make([]r3.Vec, 0, 2*len(pts2))
	// vertexStore is docs/tessellation-reach-design.md §3's deltaStore, one
	// entry per mesh vertex: the sample's own plane-local enclosure gap carried
	// through the frame (proofbound.WalkEndBoundAllow) plus the rounding the frame and
	// placement write commits (prismrecord.Payload.PointRound). Both are zero for a
	// recorded line vertex under an axis-aligned identity payload; neither is
	// ever assumed zero for anything else.
	vertexStore := make([]float64, 0, 2*len(pts2))
	vertexBudget := proofbound.NewWorkBudget(ctx)
	for j, p := range pts2 {
		if err := vertexBudget.Step(); err != nil {
			return nil, err
		}
		lo := pp.Point(p.U, p.V, pp.Z0)
		hi := pp.Point(p.U, p.V, pp.Z1)
		mesh.n.Vertices = append(mesh.n.Vertices, lo, hi)
		plane := proofbound.WalkEndBoundAllow(sampleBound[j])
		vertexStore = append(vertexStore,
			proofbound.AbsSumUpper(plane, pp.PointRound(p.U, p.V, pp.Z0, lo)),
			proofbound.AbsSumUpper(plane, pp.PointRound(p.U, p.V, pp.Z1, hi)),
		)
	}
	storeMax, err := tessellation.StoreMax(vertexStore)
	if err != nil {
		return nil, err
	}
	mesh.n.CoordBound = storeMax

	// Prove loop clearance before both caps reuse the wall rings' vertices.
	if err := tessellation.PrismCaps(ctx, &topology, sheet, capStart, capEnd,
		pp.Z0Delta, pp.Z1Delta, tessellation.RequireLoopClearance, triangulation.Triangulate); err != nil {
		return nil, err
	}
	mesh.n.Triangles, mesh.n.Source = topology.Triangles, topology.Sources

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing the windings restores outward orientation.
	if pp.Reflected() {
		for i := range mesh.n.Triangles {
			mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
		}
	}
	// Every vertex sits on the RECORDED boundary at one of the recorded sweep
	// levels — within what its own construction rounded (vertexStore) — and a
	// payload holds each level and each section only within its own displacement
	// of what it denotes: the section's in the plane
	// (docs/prism-boolean-design.md §7), each end's along the normal. So a face
	// deviates by its own trim chording plus its own store, section and axial
	// terms, which is exactly what §3's faceBound composes. The chording above
	// already paid for the section displacement out of the reserved budget, so
	// chording plus that term stays within the requested tolerance; the axial
	// displacement carries no reservation of its own and can make the complete
	// bound larger. The section displacement moves AREA too: the section can
	// differ from the one it denotes by a tube about its own boundary, charged
	// once per cap, and the boundary's own length can differ by that tube's
	// length reading, charged over the sweep height — evalPrism's own composition
	// (2·regionArea + perimeter·height), one dimension at a time. Every such term
	// is zero for a payload a caller draws.
	// The walls and, on a solid, its caps close by construction; this proves
	// the assembled mesh is watertight (or, on a sheet, manifold-with-
	// boundary) and refuses a cracked one rather than return it — the same
	// safety net the cup path already carries (core §11, never a wrong mesh).
	if err := requireMeshAudit(ctx, sheet, b, mesh); err != nil {
		return nil, err
	}
	if err := composeFaceBounds(mesh, faceTrim, faceAxial, vertexStore, pp.SectionDelta); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		// The area slack accumulated above is only part of its composition —
		// the section-displacement and per-facet terms below are the rest —
		// and the occupied-volume proof has not started. The caller asked for
		// neither, so neither is finished and neither is published
		// (tessellateContext's own withholdProofs is what drops them). Every
		// face bound this mesh states is complete and stands.
		return mesh, nil
	}
	if pp.SectionDelta > 0 {
		wallMove := proofbound.ProductUpper(proofbound.SectionDisplacementLength(pp.SectionDelta, walks), math.Abs(pp.Z1-pp.Z0))
		if sheet {
			// A sheet carries no cap, so its section-displacement area charge
			// drops both cap terms and keeps the wall's own alone.
			mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, wallMove)
		} else {
			capMove := proofbound.SectionDisplacementArea(pp.SectionDelta, walks, perimeterUpper)
			mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, capMove, capMove, wallMove)
		}
	}
	// Every coordinate the build itself computed can move each facet's own area
	// (docs/tessellation-design.md §5's per-triangle allowance), so the slack
	// carries one such term per facet beside the analytic ones above.
	mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
		tessellation.StoreAreaAllow(mesh.n.Vertices, mesh.n.Triangles, vertexStore))

	if sheet {
		// A sheet encloses no region, so there is no occupied volume to
		// prove: the mesh publishes NO volSymDiff and leaves symDiffOK
		// false (docs/surface-design.md §10). That omission IS the proof's
		// absence, not a placeholder for one still to come — operandSymDiff
		// (boolean.go) is the consumer that reads symDiffOK and refuses a
		// boolean that reaches this operand.
		return mesh, nil
	}

	// Occupied volume (docs/tessellation-reach-design.md §3). The chorded section
	// differs from the section it denotes by the circular segments it omits or
	// adds, over the sweep height; the recorded section differs from the denoted
	// one by its own displacement tube, again over that height; each end level
	// differs by its own axial displacement over the cap it caps; and every
	// computed coordinate sweeps volume at the rate of the surface it moved.
	// Absolute sums throughout — an occupied-volume bound admits no cancellation.
	height := math.Abs(pp.Z1 - pp.Z0)
	areaUpper := tessellation.FaceAreaUpper(mesh.n.Vertices, mesh.n.Triangles, mesh.n.Source, vertexStore)
	terms := []float64{
		proofbound.ProductUpper(height, segmentArea),
		proofbound.ProductUpper(proofbound.SectionDisplacementArea(pp.SectionDelta, walks, perimeterUpper), height),
		proofbound.ProductUpper(pp.Z0Delta, areaUpper[capStart]),
		proofbound.ProductUpper(pp.Z1Delta, areaUpper[capEnd]),
		proofbound.SweptVolumeAllow(storeMax, proofbound.PerturbedAreaUpper(mesh.n.Vertices, mesh.n.Triangles, storeMax)),
	}
	if err := publishSymDiff(mesh, terms); err != nil {
		return nil, err
	}
	return mesh, nil
}

// requireMeshAudit dispatches docs/tessellation-design.md §1's mandatory
// mesh audit by body kind: a BodySolid keeps §1's closed-mesh audit
// (tessellation.RequireClosedMesh, through topobuild.RequireClosedMesh)
// verbatim, and a BodySheet runs §1.2's
// manifold-with-boundary audit (requireSheetMesh) plus its own vertex-link
// safety net (tessellation.RequireSheetVertexLinks) in its place. Both audits
// live in internal/tessellation. This is the one place a surface-result build
// reaches either sheet audit, so the prism, revolve, chain and stitch sheet
// paths all get them from here.
func requireMeshAudit(ctx context.Context, sheet bool, b *Body, m *Mesh) error {
	if sheet {
		if err := requireSheetMesh(ctx, b, m); err != nil {
			return err
		}
		return topobuild.LiftAuditError(tessellation.RequireSheetVertexLinks(ctx, len(m.n.Vertices), m.n.Triangles))
	}
	return topobuild.RequireClosedMesh(&m.n)
}

// requireSheetMesh is the root side of docs/tessellation-design.md §1.2's
// free-boundary attribution audit: it numbers the body's faces in Body.Faces
// order, carries each mesh triangle's source face and the body's own recorded
// free-edge chains over as those numbers, and lets internal/tessellation
// compare the two sides. A source face the body does not carry keeps its own
// fresh number, so pointer identity decides the grouping exactly as before.
func requireSheetMesh(ctx context.Context, b *Body, m *Mesh) error {
	faces := b.Faces()
	number := make(map[*topo.Face]int, len(faces))
	for i, f := range faces {
		number[faceNode(f)] = i
	}
	source := make([]int, len(m.n.Source))
	for k, f := range m.n.Source {
		i, ok := number[f]
		if !ok {
			i = len(number)
			number[f] = i
		}
		source[k] = i
	}
	counts := freeChainCountsByFace(b)
	byNumber := make(map[int]int, len(counts))
	for f, n := range counts {
		byNumber[number[faceNode(f)]] = n
	}
	return topobuild.LiftAuditError(tessellation.RequireSheetBoundary(ctx, tessellation.SheetBoundary{
		Triangles: m.n.Triangles, SourceFaces: source, FreeChainCounts: byNumber,
	}))
}

// composeFaceBounds publishes docs/tessellation-design.md §2's sourceBound for
// every face the mesh names, as §3's sum of that face's own trim, store, section
// and axial displacements, and lifts Mesh.bound to their maximum. Every source
// face is present by construction: the walk is over the mesh's Source itself.
func composeFaceBounds(m *Mesh, trim, axial map[*topo.Face]float64, store []float64, section float64) error {
	bounds, err := tessellation.FaceBounds(m.n.Triangles, m.n.Source, trim, axial, store, section)
	if err != nil {
		return err
	}
	for f, bound := range bounds {
		topobuild.SetFaceBound(&m.n, f, bound)
	}
	return nil
}

// publishSymDiff sums an analytic payload's occupied-volume terms into the
// mesh's own volSymDiff and marks the proof complete. A term this build cannot
// state refuses (docs/tessellation-design.md §12) rather than publishing a
// symmetric-difference bound the boolean would then compose into a result.
func publishSymDiff(m *Mesh, terms []float64) error {
	total, err := tessellation.SymDiff(terms)
	if err != nil {
		return err
	}
	m.n.VolSymDiff = total
	m.n.SymDiffOK = true
	return nil
}

// tessellateCup meshes a cup (docs/modify-design.md §9, D4): the outer region O
// and the cavity region C, each with k ≥ 0 holes, sharing one rim per region
// loop at the open end. Every region loop is chorded ONCE — the same chording
// feeds its walls, its floor cap and its half of the rim band — so every ring
// vertex is shared and the mesh is watertight and consistently oriented by
// construction, exactly as the prism's is. A hole of O is a tunnel through the
// whole body; a hole of C is a solid POST rising from the floor. The planar
// faces (the kept cap over O, the pocket floor over C, and one rim band per
// loop) triangulate through the shipped cap triangulator.
// verify decides whether the area-slack and occupied-volume proofs at the end
// of the build are composed at all; every audit and every face bound above
// them runs at each level (docs/tessellation-design.md §1).
func tessellateCup(ctx context.Context, b *Body, cp cupView, chord float64, verify Verification) (*Mesh, error) {
	byRole := map[string]*topo.Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = faceNode(f)
		}
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	capStart, err := faceOfRole(provenance.RoleCapStart)
	if err != nil {
		return nil, err
	}
	shellCap, err := faceOfRole(roleShellCap)
	if err != nil {
		return nil, err
	}

	// One free-form counter for the whole chorded record — the cup's outer region
	// and its cavity are the two halves of one section (see tessellation.ChordLoop).
	work := freeform.NewFreeformWork()
	oLoops := append([]loopRecord{cp.Outer.Outer}, cp.Outer.Holes...)
	cLoops := append([]loopRecord{cp.Cavity.Outer}, cp.Cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return nil, fmt.Errorf(`%w: the cup's outer and cavity regions have different loop counts`, ErrDegenerate)
	}

	openIsMax := cp.ZOpen > cp.ZOuter
	oLo, oHi := math.Min(cp.ZOuter, cp.ZOpen), math.Max(cp.ZOuter, cp.ZOpen)
	cLo, cHi := math.Min(cp.ZCav, cp.ZOpen), math.Max(cp.ZCav, cp.ZOpen)

	mesh := bornMesh(new(Mesh))
	base := cp.BasePrism()
	var verts []r3.Vec
	// vertexStore is docs/tessellation-reach-design.md §3's deltaStore, parallel
	// to verts: the sample's plane-local enclosure gap carried through the frame
	// plus the rounding the frame/placement write commits.
	var vertexStore []float64
	add := func(v r3.Vec, store float64) int {
		verts = append(verts, v)
		vertexStore = append(vertexStore, store)
		return len(verts) - 1
	}
	// Each face's own trim and axial displacement, composed into its published
	// sourceBound once the whole cup is built.
	faceTrim := map[*topo.Face]float64{}
	faceAxial := map[*topo.Face]float64{}
	// The summed circular-segment area of each region's own loops, over the
	// region's own sweep height — the cup's occupied-volume analytic term.
	var oSegmentArea, cSegmentArea float64

	// One chorded ring per region loop: the walls hang off it and the caps and
	// rims share it, so every vertex is shared and the mesh closes by
	// construction. A ring holds its samples, the lo/hi mesh vertices of each,
	// its wall faces and the sagitta bound the chording proved.
	type ring struct {
		samples []planePoint
		loV     []int
		hiV     []int
		faces   []*topo.Face
		sag     float64
		walks   int
		perim   float64
	}
	chordRing := func(loop loopRecord, h, lo, hi, loDelta, hiDelta float64, role string, area *float64) (ring, error) {
		// A cup chords the DERIVED region loops — an offset cavity, an outer
		// contour — which no payload holds a resolution of, so each segment
		// resolves through walkOf here as it always has.
		cl, err := tessellation.ChordLoop(ctx, loop, chord, h, work, nil, 0, func(w survey2d.SideWalk) (*topo.Face, error) {
			return faceOfRole(fmt.Sprintf(role, w.Segs[0]))
		}, stationbound.ChordStationBound)
		if err != nil {
			return ring{}, err
		}
		samples := cl.Samples
		// A cup always triangulates two planar patches off this ring (its
		// kept cap or pocket floor, plus a rim band), so it keeps both cap
		// halves of the loop's slack, the same total chording's own
		// combined areaSlack used to carry before it split.
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, cl.WallSlack, cl.CapSlack, cl.CapSlack)
		*area = proofbound.AbsSumUpper(*area, cl.SegmentArea)
		r := ring{samples: point2FromRecordSlice(samples), faces: cl.FaceOf,
			sag: cl.MaxSag, walks: cl.Walks, perim: cl.PerimeterUpper}
		r.loV = make([]int, len(samples))
		r.hiV = make([]int, len(samples))
		// A wall spans both of its region's levels, so it cannot attribute its
		// axial displacement to one of them and takes the larger.
		wallAxial := math.Max(loDelta, hiDelta)
		for i, p := range samples {
			plane := proofbound.WalkEndBoundAllow(cl.BoundOf[i])
			loV := base.Point(p.U, p.V, lo)
			hiV := base.Point(p.U, p.V, hi)
			r.loV[i] = add(loV, proofbound.AbsSumUpper(plane, base.PointRound(p.U, p.V, lo, loV)))
			r.hiV[i] = add(hiV, proofbound.AbsSumUpper(plane, base.PointRound(p.U, p.V, hi, hiV)))
			f := cl.FaceOf[i]
			faceTrim[f] = math.Max(faceTrim[f], cl.SagOf[i])
			faceAxial[f] = wallAxial
		}
		return r, nil
	}
	// Each region's own lo/hi level displacement, in the order chordRing takes
	// them: the open end carries zOpenDelta, each floor its own level's.
	oLoDelta, oHiDelta := cp.ZOuterDelta, cp.ZOpenDelta
	cLoDelta, cHiDelta := cp.ZCavDelta, cp.ZOpenDelta
	if !openIsMax {
		oLoDelta, oHiDelta = cp.ZOpenDelta, cp.ZOuterDelta
		cLoDelta, cHiDelta = cp.ZOpenDelta, cp.ZCavDelta
	}

	// Outer region O: every loop in its natural sense (outer counter-clockwise,
	// holes clockwise), role side(i,j).
	oRings := make([]ring, len(oLoops))
	for i, loop := range oLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		oRings[i], err = chordRing(loop, oHi-oLo, oLo, oHi, oLoDelta, oHiDelta, fmt.Sprintf("side(%d,%%d)", i), &oSegmentArea)
		if err != nil {
			return nil, err
		}
	}

	// Cavity region C: every loop REVERSED (its material lies outside it), so
	// the reversed outer walks clockwise and each reversed hole counter-
	// clockwise (a post), role shellSide(i,j).
	cRings := make([]ring, len(cLoops))
	for i, loop := range cLoops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		rev, err := offset2d.ReverseLoopRecordContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		cRings[i], err = chordRing(rev, cHi-cLo, cLo, cHi, cLoDelta, cHiDelta, fmt.Sprintf("shellSide(%d,%%d)", i), &cSegmentArea)
		if err != nil {
			return nil, err
		}
	}
	mesh.n.Vertices = verts
	storeMax, err := tessellation.StoreMax(vertexStore)
	if err != nil {
		return nil, err
	}

	// The cup topology shares every chorded ring across its wall, floor and rim.
	toTopologyRing := func(r ring) tessellation.CupRing[*topo.Face] {
		return tessellation.CupRing[*topo.Face]{Samples: point2ToRecordSlice(r.samples),
			LoV: r.loV, HiV: r.hiV, Faces: r.faces, Sag: r.sag}
	}
	oTopology := make([]tessellation.CupRing[*topo.Face], len(oRings))
	cTopology := make([]tessellation.CupRing[*topo.Face], len(cRings))
	for i := range oRings {
		oTopology[i] = toTopologyRing(oRings[i])
	}
	for i := range cRings {
		cTopology[i] = toTopologyRing(cRings[i])
	}
	rimFace := func(i int) (*topo.Face, error) { return faceOfRole(fmt.Sprintf("rim(%d)", i)) }
	assembled, err := tessellation.AssembleCup(ctx, oTopology, cTopology, openIsMax, capStart, shellCap,
		rimFace, tessellation.RequireLoopClearance, triangulation.Triangulate, faceTrim, faceAxial,
		cp.ZOuterDelta, cp.ZCavDelta, cp.ZOpenDelta)
	if err != nil {
		return nil, err
	}
	mesh.n.Triangles, mesh.n.Source = assembled.Triangles, assembled.Sources

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing the windings restores outward orientation.
	if base.Reflected() {
		for i := range mesh.n.Triangles {
			mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
		}
	}

	// The cup's planar faces triangulate through the shipped cap triangulator,
	// which resolves every rim band — including the bridge-collinear one an
	// outward cup or a rectangular post produces, where a rounded outer loop's
	// corner tangent points land on the sharp inner loop's edge lines. The
	// walls and floors close by construction; this proves the assembled mesh is
	// watertight and refuses a cracked one rather than return it, a safety net
	// against any residual chording pathology (core §11, never a wrong mesh).
	if err := topobuild.LiftAuditError(tessellation.RequireClosedMesh(mesh.n.Triangles)); err != nil {
		return nil, err
	}
	// The offset region — the cavity inward, the outer region outward — is
	// recorded within offsetDelta of the region the cup denotes, so every face
	// it bounds carries that displacement beside its own trim, store and level
	// terms: its walls, its own planar cap and every rim. The receiver's own
	// region carries none (docs/tessellation-design.md §6).
	displacedRings, displacedCap, displacedHeight := cRings, shellCap, cHi-cLo
	if cp.Sense == cupwall.Outward {
		displacedRings, displacedCap, displacedHeight = oRings, capStart, oHi-oLo
	}
	var displacedWalks int
	var displacedPerim float64
	for _, r := range displacedRings {
		displacedWalks += r.walks
		displacedPerim = proofbound.AbsSumUpper(displacedPerim, r.perim)
	}
	if cp.OffsetDelta > 0 {
		displacedFaces := map[*topo.Face]struct{}{displacedCap: {}}
		for _, r := range displacedRings {
			for _, f := range r.faces {
				displacedFaces[f] = struct{}{}
			}
		}
		for i := range oLoops {
			rim, err := faceOfRole(fmt.Sprintf("rim(%d)", i))
			if err != nil {
				return nil, err
			}
			displacedFaces[rim] = struct{}{}
		}
		for f := range displacedFaces {
			faceTrim[f] = proofbound.AbsSumUpper(faceTrim[f], cp.OffsetDelta)
		}
	}
	if err := composeFaceBounds(mesh, faceTrim, faceAxial, vertexStore, 0); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		// The same reading the prism arm states: the slack accumulated above
		// is unfinished without the per-facet term below, and the
		// occupied-volume proof has not started, so neither is published.
		return mesh, nil
	}
	mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack,
		tessellation.StoreAreaAllow(mesh.n.Vertices, mesh.n.Triangles, vertexStore))
	// The displaced region's own area moves by its displacement area once in
	// its cap and once in the rims it bounds, and its walls' length by the
	// displacement length over their height — evalPrism's composition for a
	// section displacement, one region at a time.
	displacedArea := proofbound.SectionDisplacementArea(cp.OffsetDelta, displacedWalks, displacedPerim)
	if cp.OffsetDelta > 0 {
		wallMove := proofbound.ProductUpper(proofbound.SectionDisplacementLength(cp.OffsetDelta, displacedWalks), displacedHeight)
		mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, displacedArea, displacedArea, wallMove)
	}

	// Occupied volume (docs/tessellation-reach-design.md §3): each region's own
	// chorded-section deficit over its own sweep height, the displaced region's
	// section displacement over its height, each planar level's displacement
	// over the patch it caps, and the computed coordinates' swept volume.
	areaUpper := tessellation.FaceAreaUpper(mesh.n.Vertices, mesh.n.Triangles, mesh.n.Source, vertexStore)
	rimArea := 0.0
	for i := range oLoops {
		rim, err := faceOfRole(fmt.Sprintf("rim(%d)", i))
		if err != nil {
			return nil, err
		}
		rimArea = proofbound.AbsSumUpper(rimArea, areaUpper[rim])
	}
	terms := []float64{
		proofbound.ProductUpper(oHi-oLo, oSegmentArea),
		proofbound.ProductUpper(cHi-cLo, cSegmentArea),
		proofbound.ProductUpper(displacedHeight, displacedArea),
		proofbound.ProductUpper(cp.ZOuterDelta, areaUpper[capStart]),
		proofbound.ProductUpper(cp.ZCavDelta, areaUpper[shellCap]),
		proofbound.ProductUpper(cp.ZOpenDelta, rimArea),
		proofbound.SweptVolumeAllow(storeMax, proofbound.PerturbedAreaUpper(mesh.n.Vertices, mesh.n.Triangles, storeMax)),
	}
	if err := publishSymDiff(mesh, terms); err != nil {
		return nil, err
	}
	return mesh, nil
}

// facetedBoundError is tessellateFaceted's refusal of a chord tolerance
// finer than the bound the faceted body holds (docs/tessellation-design.md
// §7). Only a Tessellate caller, who chose the tolerance, reaches it: the
// boolean asks a restating operand at its held floor and gates the facets the
// pair touches instead (heldFloorOf, meshbool.RefuseCoarseHeldContact).
type facetedBoundError struct{ requested, held float64 }

func (e *facetedBoundError) Error() string {
	requested, minimum := units.Millimeters(e.requested), units.Millimeters(e.held)
	return fmt.Sprintf(`%v: requested tolerance %s is below the faceted body's minimum mesh bound %s; retry with a tolerance of at least %s to restate the held mesh`, ErrUnsupported, requested, minimum, minimum)
}

func (e *facetedBoundError) Unwrap() error { return ErrUnsupported }

// tessellateFaceted restates a boolean-built body's held mesh: the polygons
// ARE the boundary this evaluator holds (core §6.1), carrying their own
// proven bound. It cannot refine them — the analytic identity is gone — so a
// tolerance finer than the held bound is ErrUnsupported (facetedBoundError),
// never a mesh whose bound overstates its trust.
func tessellateFaceted(ctx context.Context, b *Body, fp facetedPayload, chord float64) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if chord < fp.meshBound {
		return nil, &facetedBoundError{requested: chord, held: fp.meshBound}
	}
	src, err := facetproof.RestateSources(ctx, topobuild.Faces(&b.n), fp.faceOf,
		len(fp.verts), len(fp.vertexBound), len(fp.tris))
	if err != nil {
		return nil, err
	}
	m := bornMesh(&Mesh{
		n: topo.Mesh{
			Vertices:   fp.verts,
			Triangles:  fp.tris,
			AreaSlack:  fp.areaSlack,
			VolSymDiff: fp.volSymDiff,
			SymDiffOK:  true,
			Source:     src,
		},
	})
	// The payload's own per-vertex record is restated unchanged, and each
	// face's bound is the largest δ(t) over its own facets
	// (docs/faceted-vertex-bounds-design.md §4.4), so the next boolean's
	// pre-pass and rim composition read each face's and each facet's own
	// figure. Their maximum is the payload's meshBound by construction.
	m.setVertexBounds(fp.vertexBound)
	if m.n.Bound != fp.meshBound {
		return nil, fmt.Errorf(`%w: a faceted payload's mesh bound is not the largest of its facet bounds`, ErrBooleanFailed)
	}
	return m, nil
}

func requireLoopClearance(ctx context.Context, pts []planePoint, loopIdx [][]int, loopSag []float64) error {
	return tessellation.RequireLoopClearance(ctx, point2ToRecordSlice(pts), loopIdx, loopSag)
}

func requireWalkClearance(ctx context.Context, pts []planePoint, loopIdx [][]int, sag [][]float64) error {
	return tessellation.RequireWalkClearance(ctx, point2ToRecordSlice(pts), loopIdx, sag)
}
