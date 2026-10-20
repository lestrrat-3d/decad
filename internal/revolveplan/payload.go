package revolveplan

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Payload is the evaluator's own record of a revolved body: the
// recorded region, the plane frame, the oriented plane-local axis, the sweep
// interval (right-handed about the axis, Phi1 > Phi0; exactly 2π apart when
// full), and the accumulated rigid placement. Every measurement and the
// whole topology derive from it, which is what makes Transform exact: it
// re-evaluates the same payload under the composed motion
// (docs/evaluator-design.md §8).
//
// Den is the ANGULAR twin of prismrecord.Payload's Z0Delta/Z1Delta: the exact angle
// each end of Phi0/Phi1 denotes (phi0Delta and phi1Delta read it). Phi0/Phi1 are
// what the resolver HELD — a float64 the extent's own resolution rounded to
// (docs/evaluator-design.md §6's "held sweep angle") — and Den is what the
// recorded AngularExtent itself DENOTES, exactly, wherever this evaluator can
// state it. A reading that folds Phi0 or Phi1 into a published measurement
// takes the per-end displacement Den proves (phi0Delta/phi1Delta,
// AngularDelta) rather than reading the held float as the truth; Den's own
// zero value is the invalid denotation, so a payload literal built without it
// (a test fixture, or an extent this evaluator cannot yet denote exactly)
// falls back to the same magnitude envelope every consumer published before
// this field existed. Being a payload field it re-evaluates with the
// payload, so Transform and Copy carry it unchanged.
//
// SurfaceResult is WithSurfaceResult's own flag (docs/surface-design.md §4):
// true when the build must publish a sheet instead of a solid. A full
// revolution mints no closing face at all, so the flag changes the face set
// only when Full is false — a partial sweep omits its two caps — while it
// changes Kind() and every measurement derived from solid vs. sheet in every
// case, full sweep included (Table W). It is part of the re-evaluable record
// for the same reason prismrecord.Payload.SurfaceResult is, so Transform and Copy
// reproduce the sheet with no further code; a plain revolve leaves
// it false. internal/sweeprecord/composite.go also builds a
// Payload literal and leaves this field at its zero value.
type Payload struct {
	Profile       momentinput.Profile
	Frame         r3.Frame
	Ax            revolveaxis.Frame
	Phi0, Phi1    float64
	Full          bool
	Den           revolveangle.Sweep
	Xform         r3.Transform
	SurfaceResult bool
	// SectionDelta is prismrecord.Payload's own §7 term over the MERIDIAN: the proven
	// upper bound on how far any recorded meridian point sits from the
	// meridian its construction denotes, and any denoted point from the
	// recorded one, both ways. Where SectionWhole is set it also states that
	// each recorded segment pairs with a denoted one of its kind whose ends
	// (and an arc's centre) sit within it, the arc's sweep taken without a 2π
	// wrap. The wall areas take it through
	// docs/surface-intersection-design.md §7.1's fold into the axis-coordinate
	// walk, the box through ExtentBoundedAlong's fifth mechanism, and the
	// solid's region readings — the Pappus volume, the centroid and a cap's
	// area — through §7.2's band (internal/revolveaxis/section_charge.go).
	// A revolve a caller draws
	// directly leaves it zero and every reading takes the path it takes today,
	// bit for bit.
	SectionDelta float64
	// SectionWhole says SectionDelta reaches every recorded coordinate — each
	// segment's two ends and an arc's centre — as an offset construction's does
	// (internal/revolveshell/shell.go), rather than only the cut ends a trim records, and
	// that the segment-wise pairing above holds. The per-walk readings then
	// charge every end, every vertex and every wall's denoted normal (§7.2); a
	// cut construction leaves it false and charges its own cut ends through
	// prismcells.TrimRevolveSegmentCharges.
	SectionWhole bool
	// RadialProof belongs to this exact profile and resolved axis. A path
	// replacing either must clear it; placement alone preserves both.
	RadialProof bool
	// BlendSegs and BlendKind are prismrecord.Payload's own blend-role descriptors,
	// read over the MERIDIAN: a revolve junction fillet or chamfer
	// (internal/brepmodify/revolve_blend.go) records, per loop, the segment indices of its
	// rewritten record that are blend connectors, and BlendKind is "fillet" or
	// "chamfer". The build gives each such wall a second kind(i,j) role beside
	// its side(i,j) one. They ride on the payload so Transform and Copy
	// re-mint the same roles; a path replacing the profile must clear them.
	BlendSegs []map[int]struct{}
	BlendKind string
	// SourceSideOrigins records the source faces of an arranged meridian wall.
	// A Boolean result retains them when Transform or Copy rebuilds its payload.
	SourceSideOrigins [][][]provenance.Ref
	// DraftCircleCaps retains the two axial cap roles when a one-sided
	// circular draft rim fillet or chamfer is rebuilt as a full revolution. In that
	// five-segment meridian, side(0,1) is the far cap and side(0,4) the
	// near cap. Placement re-evaluates this same meridian and roles.
	DraftCircleCaps bool
	// DraftCircleShellRoles stamps a drafted circular cup's kept cap, open
	// rim, cavity floor and cavity wall after the meridian revolution.
	DraftCircleShellRoles bool
}

// RequireExactSection is the reject-only guard an operation that
// rewrites the recorded meridian takes — a junction blend, a shell: each
// offsets or cuts the RECORDED meridian, and a meridian displaced from the one
// it denotes has no proven rewrite. The field is either zero or it is not, so
// the guard needs no tolerance and can only refuse.
func RequireExactSection(rp Payload, what string) error {
	if rp.SectionDelta == 0 {
		return nil
	}
	return fmt.Errorf(
		`%w: %s cannot integrate a region over a meridian carrying its own section displacement of %v mm`,
		decaderr.ErrUnsupported, what, rp.SectionDelta)
}

// Meridian is a *view* of rp as a prismrecord.Payload carrying the recorded MERIDIAN
// and the frame and placement it is expressed in — never a body this evaluator
// builds from. It is what docs/surface-intersection-design.md §3.1 hands
// prismcells.BuildScene, prismcells.NewReexpression, prismcells.Classify and the rest of
// §3's resolution, every one of which reads a profile, a frame and a placement
// and nothing else. The sweep fields are deliberately absent: the revolve's own
// angular interval is S6's business, never the private 2D scene's, and the
// levels a prismrecord.Payload would carry have no meaning for a meridian.
func (rp Payload) Meridian() prismrecord.Payload {
	return prismrecord.Payload{Profile: rp.Profile, Frame: rp.Frame, Xform: rp.Xform}
}

// Transform is the accumulated rigid placement.
func (rp Payload) Transform() r3.Transform { return rp.Xform }

// ProvesSimple implements topo.SimpleProver through revolveaxis.ProvesSimple
// (its comment holds the argument). A revolve's build runs no crossing audit
// over a triangle set at all, so the proof is a different shape than the
// prism's or loft's, not merely a weaker version of it.
func (rp Payload) ProvesSimple(ctx context.Context) bool {
	return revolveaxis.ProvesSimple(ctx, rp.Profile, rp.Ax, rp.Full, rp.RadialProof,
		rp.SectionDelta, prismextent.ProfileBoundaryExtremes)
}

// Placed re-evaluates the same record under the composed motion.
func (rp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	rp.Xform = composed
	return Build(ctx, env, rp, freeform.NewFreeformWork())
}

// Lift is the record rp's sweep basis is built from: the plane frame and the
// resolved axis in it.
func (rp Payload) Lift() revolvemesh.RevolveLift {
	return revolvemesh.RevolveLift{Frame: rp.Frame, AU: rp.Ax.AU, AV: rp.Ax.AV, DU: rp.Ax.DU, DV: rp.Ax.DV}
}

// Basis derives the sweep basis from the plane frame and the axis frame.
func (rp Payload) Basis() revolvemesh.RevolveBasis { return rp.Lift().Basis() }

// AxisBound is the resolved axis's own proven anchor and direction
// displacement, in the form the swept-vertex comparison reads it.
func (rp Payload) AxisBound() revolvemesh.AxisBound {
	return revolvemesh.AxisBound{AU: rp.Ax.AUBound, AV: rp.Ax.AVBound, DU: rp.Ax.DUBound, DV: rp.Ax.DVBound}
}

// sweptPoint is the recorded plane-local point a swept vertex denotes, with
// the walk's own proven bound on it (survey2d.SegmentWalk's StartBound/
// EndBound: zero where the record states the coordinate, the evaluation's
// own bound where the walk computed it).
type sweptPoint struct {
	u, v  float64
	bound proofbound.WalkEndBound
}

// denotedPoint widens a recorded point to the meridian it denotes.
func (rp Payload) denotedPoint(p sweptPoint) sweptPoint {
	charge := revolveaxis.SectionWholeCharges(rp.SectionWhole, rp.SectionDelta)
	p.bound = revolveaxis.DenotedBound(p.bound, charge)
	return p
}

// ChargedWalk re-expresses a segment in axis coordinates and charges either
// the whole displaced section or the endpoints cut by a trim.
func (rp Payload) ChargedWalk(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk) (survey2d.SegmentWalk, error) {
	if c := revolveaxis.SectionWholeCharges(rp.SectionWhole, rp.SectionDelta); c.U != 0 {
		walk := rp.Ax.WalkCharged(w, c, c)
		return revolveaxis.ChargeWholeWalk(walk, rp.SectionDelta, rp.Ax.DU, rp.Ax.DV), nil
	}
	startCharge, endCharge, err := prismcells.TrimRevolveSegmentCharges(seg, rp.SectionDelta)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	return rp.Ax.WalkCharged(w, startCharge, endCharge), nil
}

// walkStart and walkEnd read a PLANE-local walk's (revolveaxis.ResolvedWalks.Plane) two
// ends, each bounded against the point its recorded segment seg denotes there
// (boundarywalk.DenotedStartBound and DenotedEndBound).
func walkStart(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk) sweptPoint {
	return sweptPoint{u: w.StartU, v: w.StartV, bound: boundarywalk.DenotedStartBound(seg, w)}
}

func walkEnd(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk) sweptPoint {
	return sweptPoint{u: w.EndU, v: w.EndV, bound: boundarywalk.DenotedEndBound(seg, w)}
}

// revolveJunctionStart reads the junction where axis walk prev ends and next starts
// as a plane-local point bounded against the points BOTH neighbours' recorded
// segments denote there (boundarywalk.JunctionVertex): at a cut junction the
// two differ (docs/evaluator-design.md §4).
func revolveJunctionStart(rw revolveaxis.ResolvedWalks, prev, next survey2d.SideWalk) sweptPoint {
	pi, ni := prev.Segs[len(prev.Segs)-1], next.Segs[0]
	u, v, bound := boundarywalk.JunctionVertex(rw.Segs[pi], rw.Plane[pi], rw.Segs[ni], rw.Plane[ni])
	return sweptPoint{u: u, v: v, bound: bound}
}

// sweptEnd is one end of the sweep a vertex sits at: the held angle and the
// angle the record denotes there (rp.Phi0 with rp.Den.Phi0, or rp.Phi1 with
// rp.Den.Phi1).
type sweptEnd struct {
	phi float64
	den revolveangle.Angle
}

func (rp Payload) end0() sweptEnd { return sweptEnd{phi: rp.Phi0, den: rp.Den.Phi0} }
func (rp Payload) end1() sweptEnd { return sweptEnd{phi: rp.Phi1, den: rp.Den.Phi1} }

// sweptGap proves how far held sits from the point at denotes, swept to the
// angle end's record states (revolvemesh.RevolveLift.SweptPointGap;
// topology.go's Vertex.Position contract). An end with no denotation — a
// ToFaceAngular stop, or a payload literal built without one — states no
// angle to rotate to, so the gap is unbounded there, which is the +Inf the
// end's own displacement (phi0Delta/phi1Delta) answers for every reading.
func (rp Payload) sweptGap(held r3.Vec, at sweptPoint, end sweptEnd) float64 {
	if !end.den.Valid() {
		return math.Inf(1)
	}
	sin, cos, ok := end.den.SinCosFor(end.phi)
	if !ok {
		return math.Inf(1)
	}
	return rp.Lift().SweptPointGap(rp.AxisBound(), rp.Xform, at.u, at.v, at.bound, sin, cos, held)
}

// sweptVertex places one swept vertex at axial z and radius rho — the
// re-expressed, possibly snapped axis coordinates the walk carries — swept to
// the first of ends, and stamps it with denot, the zero token where the vertex
// mints no curve certificate. Its bound is ONE exact comparison per end
// against the recorded point at, rotated to the angle that end denotes
// (sweptGap): the re-expression's rounding, the snap, the axis's own anchor
// and direction error, the angular displacement and trigonometric evaluation,
// and the frame lift and placement rounding all sit inside it. A vertex one
// end alone places passes that end; the shared on-axis vertex a partial sweep
// gives both caps passes both, since it stands for the recorded point at each.
// The bound is zero wherever every comparison is exact, which keeps an
// ordinary revolve's φ = 0 cap and seam vertices Exact.
func (rp Payload) sweptVertex(alloc topo.Alloc, b revolvemesh.RevolveBasis, z, rho float64, at sweptPoint, denot denotation.CurveToken, ends ...sweptEnd) *topo.Vertex {
	held := rp.point(b, z, rho, ends[0].phi)
	gap := 0.0
	for _, end := range ends {
		gap = math.Max(gap, rp.sweptGap(held, at, end))
	}
	return topobuild.NewVertex(alloc, topo.Vertex{Position: held, Bound: units.Millimeters(gap), Denot: denot})
}

// frameCharge is massmoment.MapCharge over the map the record denotes
// through: the plane-coordinate solid of revolution carried through
// L = B·[U V U×V] (massmoment.PlaneMap), the leaves sweptGap reads. Its third
// column is the exact cross product because the sweep's own E1 = W × E0 is.
// The centroid owes nothing: an affine map carries a centroid to the centroid
// of the image, so lifting the plane-coordinate centroid through L is exact.
func (rp Payload) frameCharge() (massmoment.MapCharge, error) {
	m, err := massmoment.PlaneMap(rp.Frame, rp.Xform)
	if err != nil {
		return massmoment.MapCharge{}, err
	}
	return massmoment.MapChargeOf(m)
}

// point places the axis-frame point (z, ρ) at sweep angle φ into placed
// world space.
func (rp Payload) point(b revolvemesh.RevolveBasis, z, rho, phi float64) r3.Vec {
	return rp.Lift().Point(b, rp.Xform, z, rho, phi)
}

// Reflected reports whether the accumulated placement flips handedness — a
// reflected solid's rotational senses invert with it, and every swept edge's
// axis carries the corrected sign.
func (rp Payload) Reflected() bool { return rp.Xform.IsReflection() }

// JunctionCircle is the placed circle a junction at axis coordinates (z, ρ)
// sweeps: its centre on the axis at z, its axis the placed sweep axis (negated
// under a reflected placement, so the circle's sense stays the sweep's), and
// its radius ρ. buildRevolveLoop stamps every junction edge from it, and
// internal/brepmodify/revolve_blend.go matches a selected edge against it, so the two read the
// same numbers from the same arithmetic.
func (rp Payload) JunctionCircle(b revolvemesh.RevolveBasis, z, rho float64) (r3.Vec, r3.Vec, units.Value) {
	sweepSign := 1.0
	if rp.Reflected() {
		sweepSign = -1
	}
	return rp.point(b, z, 0, 0), rp.Xform.ApplyDir(b.W).Scale(sweepSign), units.Millimeters(rho)
}

// latitudeCurveBound is an Edge's curveBound for the latitude circle or the
// junction arc the recorded point at sweeps (revolvemesh.RevolveLift.LatitudeGap).
func (rp Payload) latitudeCurveBound(at sweptPoint, center, axis r3.Vec, radius float64) (float64, bool) {
	gap := rp.Lift().LatitudeGap(rp.AxisBound(), rp.Xform, at.u, at.v, at.bound, center, axis, radius)
	if proofbound.IsNonFinite(gap) || !(proofbound.ProductUpper(2, gap) < radius) {
		return 0, false
	}
	return gap, true
}

// WallSurface is the placed surface of revolution one off-axis walk sweeps,
// and whether the face's outward (material-leaving) normal is the surface's
// geometric normal negated. The walk runs with the region's material on its
// left in (z, ρ), so its outward direction is the tangent rotated a quarter
// turn right: (t_ρ, −t_z). A cylinder's and a cone's geometric normal has a
// positive ρ component, so those reverse exactly when the tangent climbs
// (t_z > 0 — a hole wall, or the inner wall of an annular section); a
// sphere's and a torus's geometric normal points away from the generating
// circle's center, which a counter-clockwise walk's outward direction is —
// so those reverse exactly when the walk runs clockwise. Radial directions
// are reflection-equivariant, so a reflected placement changes none of
// this; only the plane frames (built from cross products) correct for it.
func (rp Payload) WallSurface(b revolvemesh.RevolveBasis, w survey2d.SegmentWalk, kind revolveaxis.WallKind) (surfacegeom.Surface, bool, error) {
	place := func(z float64) r3.Vec { return rp.Xform.Apply(b.A3.Add(b.W.Scale(z))) }
	axis := rp.Xform.ApplyDir(b.W)
	switch kind {
	case revolveaxis.WallCylinder:
		return surfacegeom.Cylinder{
			Origin: place(w.StartU),
			Axis:   axis,
			Radius: units.Millimeters((w.StartV + w.EndV) / 2),
		}, w.TanInU > 0, nil
	case revolveaxis.WallPlane:
		// The outward normal is ±axis by the walk's radial heading; the
		// frame's axes are ordered so its normal is outward, swapped once
		// more under a reflected placement.
		u3, v3 := b.E0, b.E1
		if w.TanInV < 0 {
			u3, v3 = v3, u3
		}
		if rp.Reflected() {
			u3, v3 = v3, u3
		}
		f, err := r3.NewFrame(place(w.StartU), rp.Xform.ApplyDir(u3), rp.Xform.ApplyDir(v3))
		if err != nil {
			return nil, false, fmt.Errorf(`%w: the placed wall frame is degenerate: %s`, decaderr.ErrDegenerate, err)
		}
		return surfacegeom.Plane{Frame: f}, false, nil
	case revolveaxis.WallCone:
		// The apex is where the wall meets the axis; the cone's radius
		// grows along its stored Axis direction.
		dz, dr := w.EndU-w.StartU, w.EndV-w.StartV
		apex := w.StartU - w.StartV*dz/dr
		growth := 1.0
		if dz*dr < 0 {
			growth = -1
		}
		return surfacegeom.Cone{
			Origin:    place(apex),
			Axis:      axis.Scale(growth),
			Radius:    units.Millimeters(0),
			HalfAngle: units.Radians(math.Atan2(math.Abs(dr), math.Abs(dz))),
		}, w.TanInU > 0, nil
	case revolveaxis.WallSphere:
		return surfacegeom.Sphere{
			Center: place(w.CU),
			Radius: units.Millimeters(w.Radius),
		}, w.Th1 < w.Th0, nil
	case revolveaxis.WallTorus:
		return surfacegeom.Torus{
			Center: place(w.CU),
			Axis:   axis,
			Major:  units.Millimeters(w.CV),
			Minor:  units.Millimeters(w.Radius),
		}, w.Th1 < w.Th0, nil
	case revolveaxis.WallFreeform:
		return surfacegeom.NURBSSurface{}, false, nil
	default:
		return nil, false, fmt.Errorf(`%w: a wall on the axis sweeps no surface`, decaderr.ErrDegenerate)
	}
}

// denotationInput is what revolveaxis.DenotationInput reads off rp to
// enclose the surfaces its walls and caps denote.
func (rp Payload) denotationInput() revolveaxis.DenotationInput {
	return revolveaxis.DenotationInput{Lift: rp.Lift(), Axis: rp.AxisBound(), Xform: rp.Xform,
		SectionWhole: rp.SectionWhole, SectionDelta: rp.SectionDelta}
}

// phi0Delta and phi1Delta are the proven per-end displacements this payload's
// held sweep carries — the angular twins of prismrecord.Payload.Z0Delta/Z1Delta,
// read off the payload's own denotation against its own held float rather
// than stored redundantly, since the payload already carries both.
func (rp Payload) phi0Delta() float64 { return rp.Den.Phi0.Delta(rp.Phi0) }
func (rp Payload) phi1Delta() float64 { return rp.Den.Phi1.Delta(rp.Phi1) }

// AngularDelta is the larger of the two ends' displacements: the figure a
// reading that cannot attribute its error to one particular end takes,
// mirroring prismrecord.Payload.AxialDelta().
func (rp Payload) AngularDelta() float64 { return math.Max(rp.phi0Delta(), rp.phi1Delta()) }

// Sweep is the proven bound on the sweep width every mass and edge reading
// multiplies into its own quantity: the held float64 subtraction stays the
// published value exactly as it always has, and the bound is the smaller of
// the magnitude envelope proofbound.ConservativeValueError has always published here
// (internal/proofbound/bounded.go's own documented fallback, sound for any legal sweep since a
// legal sweep never exceeds a full turn) and the proven displacement between
// that held value and the sweep the record DENOTES (Den.WidthInterval),
// wherever a denotation exists for both ends. math.Min follows
// internal/proofbound/bounded.go's own convention for a reading a certified bracket admits — it
// can only shrink the published bound, never widen it — so a ToFaceAngular
// sweep or any other denotation this file cannot state keeps exactly the
// envelope it always published.
func (rp Payload) Sweep() proofbound.BoundedScalar {
	held := proofbound.BoundedSub(proofbound.ExactScalar(rp.Phi1), proofbound.ExactScalar(rp.Phi0))
	fallback := proofbound.ConservativeValueError(held.Value, proofbound.TwoPiUpper())
	enc, ok := rp.Den.WidthInterval()
	if !ok {
		return proofbound.MeasuredScalar(held.Value, fallback)
	}
	return proofbound.MeasuredScalar(held.Value, math.Min(fallback, proofbound.IntervalFloatError(enc, held.Value)))
}

// ChainPayload is chain Revolve's own record of a shell body: the
// recorded open walk SET, the plane frame, the oriented plane-local axis, the
// sweep interval, and the accumulated rigid placement — ChainPayload
// is to chain Revolve what chainrecord.Payload is to chain Extrude, and what
// Payload is to Revolve (docs/surface-design.md §13.4). A plain
// chain Revolve builds the one-walk case; docs/surface-intersection-design.md
// §3.4 is what first builds more than one, one lump per surviving walk, and
// sets SectionDelta beside them.
type ChainPayload struct {
	Chains     []sectionrecord.ChainRecord
	Frame      r3.Frame
	Ax         revolveaxis.Frame
	Phi0, Phi1 float64
	Full       bool
	Den        revolveangle.Sweep
	Xform      r3.Transform
	// SectionDelta is prismrecord.Payload's own §7 term (docs/prism-boolean-design.md),
	// carried here on the identical terms: the proven upper bound on how far
	// any recorded meridian coordinate sits from the meridian its construction
	// denotes. It is zero for every walk a caller draws through chain Revolve,
	// and docs/surface-intersection-design.md §7.1 derives how a nonzero one
	// reaches this body's published Area and Bounds.
	SectionDelta float64
}

// Revolve is a *view* of rp as a Payload wrapping the chain's own
// segment list as an unclosed "outer loop" with no holes — never a body this
// evaluator builds from directly — used only to feed the axis, sweep and
// extent readings Payload already proves generically over rp.Profile:
// none of them assumes the loop closes (docs/surface-design.md §13.4).
// SurfaceResult stays false and is read nowhere below: chain Revolve never
// runs Payload's own solid/cap topology, only its own open-walk build
// (buildChainRevolveWalls).
// The first chain stands in for a profile's outer loop and the rest for its
// holes, exactly as chainrecord.Payload.Prism folds its own walk set: every reading
// this view feeds walks the segments and cares about neither winding nor
// closure.
func (rp ChainPayload) Revolve() Payload {
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord(rp.Chains[0])}
	for _, c := range rp.Chains[1:] {
		profile.Holes = append(profile.Holes, sectionrecord.LoopRecord(c))
	}
	return Payload{
		Profile: profile,
		Frame:   rp.Frame,
		Ax:      rp.Ax,
		Phi0:    rp.Phi0, Phi1: rp.Phi1,
		Full:         rp.Full,
		Den:          rp.Den,
		Xform:        rp.Xform,
		SectionDelta: rp.SectionDelta,
	}
}

// WalkView is the single-walk view the wall build takes: chain ci alone as the
// outer loop, so prismrecord.SideOrigins's segment indices and revolvemass.WallAxisMoment's own
// rp.Profile.Outer read reach exactly that walk's segments. loopIdx feeds the
// face role, so two walks' faces never collide on one role string.
func (rp ChainPayload) WalkView(ci int) Payload {
	view := rp.Revolve()
	view.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord(rp.Chains[ci])}
	return view
}

// Transform is the accumulated rigid placement.
func (rp ChainPayload) Transform() r3.Transform { return rp.Xform }

// ProvesSimple implements topo.SimpleProver. A chain-fed revolve earns leg 4
// on the identical full-turn argument, re-read over the chain's own open walk
// through the SAME generic reading Payload.ProvesSimple already takes off
// Profile and Ax — neither of which assumes the walk closes
// (docs/surface-design.md §13.4).
func (rp ChainPayload) ProvesSimple(ctx context.Context) bool {
	return rp.Revolve().ProvesSimple(ctx)
}

// Placed re-evaluates the same record under the composed motion (core §8). A
// re-evaluation path: no preflight has run on this record within the call, so
// the build opens the record's one free-form work counter itself
// (docs/spline-design.md §5.2), exactly as Payload.Placed does.
func (rp ChainPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	rp.Xform = composed
	return BuildChain(ctx, env, rp, freeform.NewFreeformWork())
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "revolvePayload" }

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish): the root type name this payload had before it moved.
func (ChainPayload) SurveyClass() string { return "chainRevolvePayload" }
