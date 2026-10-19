package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/alledgechamfermesh"
	"github.com/lestrrat-3d/decad/internal/brepbandmesh"
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/cuptess"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/patchmesh"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stitchtess"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file owns the public mesh, tessellation dispatch, and prism mesh handle
// adapter (core §11, docs/evaluator-design.md §9).
// A boolean-faceted body restates its held mesh here; the exact-predicate
// mesh boolean that produces it lives in boolean_mesh.go.

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
// withholds, such as prism, brep, cup, revolve, cap-loop chamfer and draft.
// A restatement path (faceted, loft, all-planar stitch) copies its proof terms
// off the payload at no cost and publishes them unconditionally;
// tessellateContext withholds them afterwards.
func tessellateBodyContext(ctx context.Context, b *Body, chord float64, verify Verification) (*Mesh, error) {
	formatSurface := func(s surfacegeom.Surface) any { return publicSurface(s) }
	if p, ok := b.n.Payload.(allEdgeChamferPayload); ok {
		node, err := alledgechamfermesh.Build(ctx, &b.n, p, chord,
			verify >= VerifyBoundary, verify >= VerifyAll)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok {
		node, err := facetedrecord.Restate(ctx, &b.n, fp, chord)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if cp, ok := b.n.Payload.(cupPayload); ok {
		node, err := cuptess.Tessellate(ctx, &b.n, cp.View(), chord, verify >= VerifyAll)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if sp, ok := b.n.Payload.(stackedPrismPayload); ok {
		node, err := stackedrecord.Tessellate(ctx, &b.n, sp, chord, verify >= VerifyAll)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if bp, ok := b.n.Payload.(brepPayload); ok {
		node, err := brepbandmesh.Tessellate(ctx, &b.n, bp, chord, verify >= VerifyAll)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if lp, ok := b.n.Payload.(loftPayload); ok {
		// The loft path exactly restates the payload's complete set for a
		// solid or its recorded wall range for a sheet, with no chording
		// (internal/loftmesh.RestateLoft's own doc comment owns why).
		node, err := loftrecord.Tessellate(ctx, &b.n, lp)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if tp, ok := b.n.Payload.(twistedSweepPayload); ok {
		if chord < tp.Delta {
			return nil, fmt.Errorf(`%w: the twisted sweep's held boundary is %g mm from its true wall; %g mm is finer`, ErrUnsupported, tp.Delta, chord)
		}
		if tp.Mesh != nil {
			mesh := bornMesh(&Mesh{n: *tp.Mesh})
			return mesh, nil
		}
		node, err := loftrecord.Tessellate(ctx, &b.n, tp.Held, tp.SourceRoles)
		if err != nil {
			return nil, err
		}
		for _, face := range node.Source {
			topobuild.SetFaceBound(node, face, tp.Delta)
		}
		node.Bound = tp.Delta
		node.AreaSlack = tp.AreaSlack
		node.VolSymDiff = tp.VolSymDiff
		node.SymDiffOK = true
		return bornMesh(&Mesh{n: *node}), nil
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
		node, err := patchmesh.Profile(ctx, &b.n, pp, chord, patchmesh.Level(verify))
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if rp, ok := b.n.Payload.(revolvePayload); ok {
		node, err := revolvetess.TessellateRevolve(ctx, &b.n, rp, chord, revolvetess.Level(verify))
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if cbp, ok := b.n.Payload.(capBlendPayload); ok {
		return tessellateCapBlend(ctx, b, cbp, chord, verify)
	}
	if dp, ok := b.n.Payload.(draftPayload); ok {
		node, err := draftrecord.Tessellate(ctx, &b.n, dp, chord, verify >= VerifyAll)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if dp, ok := b.n.Payload.(twoSidedDraftPayload); ok {
		node, err := draftrecord.TessellateTwoSided(ctx, &b.n, envOf(b.n.Doc, b.n.Origin.Producer), dp, chord, verify >= VerifyAll)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if sp, ok := b.n.Payload.(stitchPayload); ok {
		var node *topo.Mesh
		var err error
		if sp.Tris == nil {
			node, err = stitchtess.Curved(ctx, &b.n, sp, chord, revolvetess.Level(verify), formatSurface)
		} else {
			// The all-planar stitch restatement takes no chord tolerance.
			node, err = stitchtess.Restate(ctx, &b.n, sp, formatSurface)
		}
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if pp, ok := b.n.Payload.(bodyPatchPayload); ok {
		return tessellateBodyPatch(ctx, b, pp, chord, verify)
	}
	if cp, ok := b.n.Payload.(chainPayload); ok {
		// The chain-fed ribbon path takes no chord tolerance either, on the
		// identical reasoning: every wall is already an exact planar quad,
		// so there is no chording decision for a tolerance to bind
		// (internal/chainrecord.Tessellate's own doc comment owns why).
		node, err := chainrecord.Tessellate(ctx, &b.n, cp, formatSurface)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
	}
	if sp, ok := b.n.Payload.(chainSweepPayload); ok {
		if len(sp.Spans) != 0 {
			node, err := revolvetess.TessellateChainSweep(ctx, &b.n, sp, chord, revolvetess.Level(verify))
			if err != nil {
				return nil, err
			}
			return bornMesh(&Mesh{n: *node}), nil
		}
		if sp.Arc {
			node, err := revolvetess.TessellateChainArcSweep(ctx, &b.n, sp, chord, revolvetess.Level(verify))
			if err != nil {
				return nil, err
			}
			return bornMesh(&Mesh{n: *node}), nil
		}
		// the chain Sweep's one-span straight reduction builds through the
		// identical chainrecord.Build a plain chain Extrude does, over
		// the chainPayload it wraps unchanged (internal/sweeprecord's
		// FinishChain), so it reuses that same restatement rather
		// than a second one keyed on the wrapper type.
		node, err := chainrecord.Tessellate(ctx, &b.n, sp.Chain, formatSurface)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *node}), nil
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
			node, err := revolvetess.TessellateRevolveWithRoles(
				ctx, &b.n, sp.Revolve, chord, revolvetess.Level(verify), sweepArcRole(sp.ReverseArcCaps))
			if err != nil {
				return nil, err
			}
			return bornMesh(&Mesh{n: *node}), nil
		}
		return tessellatePrismWithCountFloor(ctx, b, sp.Prism, sweepWallRole, chord, verify, nil, 0)
	}
	pp, ok := b.n.Payload.(prismPayload)
	if !ok {
		// Chording is per payload kind. Name both the staged kind and the
		// implemented set so the refusal cannot misstate evaluator reach.
		return nil, fmt.Errorf(`%w: tessellation does not support payload %s; supported payload classes are prism, patch, stacked prism, brep, chain-fed prism, sweep, mitred sweep, coil, revolve, cup, loft, cap-loop chamfer, draft (tapered extrude), stitch, and faceted`, ErrUnsupported, reportvocab.PayloadTypeName(b.n.Payload))
	}
	return tessellatePrismWithCountFloor(ctx, b, pp, prismWallRole, chord, verify, nil, 0)
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

// tessellatePrismWithCountFloor births the prism mesher's node in its root
// handle, so the one-entry tessellation cache can return that same handle.
func tessellatePrismWithCountFloor(ctx context.Context, b *Body, pp prismPayload,
	wallRole func(loop, seg int) string, chord float64, verify Verification,
	countFloor func(loop, seg int) int, freeformTarget float64) (*Mesh, error) {
	node, err := prismtess.Build(ctx, &b.n, pp, wallRole, chord,
		verify >= VerifyAll, countFloor, freeformTarget)
	if err != nil {
		return nil, err
	}
	return bornMesh(&Mesh{n: *node}), nil
}

// requireMeshAudit dispatches docs/tessellation-design.md §1's mandatory
// mesh audit by body kind. A solid takes the closed-mesh audit; a sheet takes
// topobuild.RequireSheetMesh's free-boundary and vertex-link audits. Angular
// Revolve and chain Sweep builders call that node audit directly.
func requireMeshAudit(ctx context.Context, sheet bool, b *Body, m *Mesh) error {
	if sheet {
		return topobuild.RequireSheetMesh(ctx, &b.n, &m.n)
	}
	return topobuild.RequireClosedMesh(&m.n)
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

// tessellateBodyPatch binds source handles and public refusal names to the
// PatchEdges node builder. The circular source mesh is certified first at
// VerifyBoundary even when the requested result level is VerifyNone.
func tessellateBodyPatch(ctx context.Context, b *Body, pp bodyPatchPayload,
	chord float64, verify Verification,
) (*Mesh, error) {
	var node *topo.Mesh
	var err error
	if prism, ok := patchchain.CircularPrismSource(pp); ok {
		source, sourceErr := tessellatePrismWithCountFloor(ctx, bodyHandle(pp.Faces[0].Body), prism,
			prismWallRole, chord, VerifyBoundary, nil, 0)
		if sourceErr != nil {
			return nil, sourceErr
		}
		node, err = patchmesh.CircularPrism(ctx, &b.n, pp, prism, &source.n, chord, patchmesh.Level(verify))
	} else {
		node, err = patchmesh.Planar(ctx, &b.n, pp, patchmesh.Level(verify),
			func(s surfacegeom.Surface) any { return publicSurface(s) },
			func(c surfacegeom.Curve) any { return publicCurve(c) })
	}
	if err != nil {
		return nil, err
	}
	return bornMesh(&Mesh{n: *node}), nil
}
