package decad

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessaudit"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file owns the public mesh, tessellation dispatch, and prism mesh handle
// adapter (core §11, agent-docs/evaluator-design.md §9).
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
	n         topo.Mesh
	queryOnce sync.Once
	queryData *pointquery.PreparedMesh
}

// Vertices returns the mesh vertex positions in millimetres (core §5.2).
// Every vertex lies on the held boundary. Its deviation from the denoted body,
// including inherited payload displacement, is covered by Bound.
func (m *Mesh) Vertices() []r3.Vec { return append([]r3.Vec(nil), m.n.Vertices...) }

// Triangles returns the facets as index triples into Vertices, wound
// counter-clockwise seen from outside the body. On a [BodySheet] there is no
// "outside": the winding is counter-clockwise seen from the shell's own
// positive side (agent-docs/surface-design.md §2.3) — the same side [Face.NormalAt]
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
// (agent-docs/evaluator-design.md §9). The elements are the body's live faces.
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
// own Bézier chain, measured against agent-docs/spline-design.md §6.2.1's sagitta,
// and refuses rather than coarsen when its exact arithmetic passes the fixed
// free-form work budget or when the chain would carry more chords than one
// curve may.
//
// Two things follow from the chording sagitta being PROVEN rather than tight
// (agent-docs/tessellation-design.md §3 derives it; tessellation.ChordSagitta implements it).
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
// [ErrUnsupported] (agent-docs/sweep-design.md Table D row D2).
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
// A tapered extrude (a draft body, agent-docs/draft-design.md) meshes as that same
// band with no straight slab: one ruled wall per section walk between the near
// cap's rim and the far section, a sharp miter at every line-line corner,
// convex or reflex, and both caps triangulated. Where every circular wall
// meets its neighbours exactly tangent, or meets a line or an arc tangent up
// to a turn whose sine is at most 2⁻³², a sliver the proof charges
// (agent-docs/draft-design.md §9.1.1), it also proves the volume it and the
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
// (agent-docs/prism-boolean-design.md §7); that displacement is reserved from tol
// before any chord is chosen, so a tol it exhausts is [ErrUnsupported] too.
// A body this evaluator did not build at all is also [ErrUnsupported].
//
// A [BodySheet] built by a surface-result prism or revolve
// (`WithSurfaceResult`) meshes its walls exactly as the solid the same
// record would have built. A prism sheet, and a partial-sweep revolve
// sheet, omit both caps: no cap triangulation runs and no cap face appears
// in SourceFaces. A full-turn revolve mints no cap in either kind, so its
// sheet mesh is bit-identical to the solid's and carries no free edge at
// all. The mandatory audit runs agent-docs/tessellation-design.md §1.2's
// manifold-with-boundary check in place of the closed-mesh audit a solid
// takes, over the same directed-edge structure — with no free edge, the two
// audits agree with no arm of their own. The signed-volume orientation
// check runs only when the mesh is closed — a solid, or a full-turn sheet —
// and never on an open partial-sweep sheet, where that sum is
// anchor-dependent and decides nothing. A sheet's areaSlack drops the cap
// terms it no longer carries, and it publishes no occupied-volume proof at
// all — [Union], [Cut] and [Intersect] refuse a sheet operand outright
// (agent-docs/surface-design.md Table X), so the absence costs nothing a caller
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
// a level below VerifyAll withholds agent-docs/tessellation-design.md §2's area and
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
	node, err := tessdispatch.Body(ctx, &b.n, chord, tessdispatch.Verify(verify), tessdispatch.Options{
		Surface: func(s surfacegeom.Surface) any { return publicSurface(s) },
		Curve:   func(c surfacegeom.Curve) any { return publicCurve(c) },
		Alloc:   handleAlloc{},
	})
	if err != nil {
		return nil, err
	}
	// agent-docs/tessellation-design.md §4: every outer shell winds outward and
	// every void shell into its cavity, at every verification level.
	if err := topobuild.RequireShellWinding(ctx, node, b.n.Lumps); err != nil {
		return nil, err
	}
	mesh := bornMesh(&Mesh{n: *node})
	if verify < VerifyAll {
		mesh.withholdProofs()
	}
	mesh.n.BoundaryOK = verify >= VerifyBoundary || !tessaudit.AuditsFacetContact(b.n.Payload)
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	b.n.Tess.Store(&topo.TessEntry{ChordBits: key, Verify: int(verify), Mesh: meshNode(mesh)})
	return mesh, nil
}

// VerifyNone is the same tessellation at VerifyNone (verifiedmesh.DraftMesher).
func (verifyAllMesher) VerifyNone(ctx context.Context, b *topo.Body, tolMM float64) (*topo.Mesh, error) {
	m, err := tessellateContext(ctx, bodyHandle(b), units.Millimeters(tolMM), VerifyNone)
	if err != nil {
		return nil, err
	}
	return &m.n, nil
}

// verifyAllMesher gives the internal builders a body's mesh at VerifyAll
// (internal/verifiedmesh), tessellated as Body.Tessellate would.
type verifyAllMesher struct{}

func (verifyAllMesher) VerifyAll(ctx context.Context, b *topo.Body, tolMM float64) (*topo.Mesh, error) {
	m, err := tessellateContext(ctx, bodyHandle(b), units.Millimeters(tolMM), VerifyAll)
	if err != nil {
		return nil, err
	}
	return &m.n, nil
}
