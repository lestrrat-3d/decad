package revolveplan

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemass"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// This file builds a revolve's body from its Payload: the wall surface
// each recorded segment sweeps, the two cap faces a partial sweep closes with,
// the poles and seams a full sweep instead joins, and the measurements the
// finished body publishes.
//
// Build is the whole build and states the order.
// buildRevolveLoop is the per-loop walk, and revLoopParts is what it hands
// back so a full sweep and a partial one assemble the same parts differently
// rather than through two builds. Every face carries the bound its own
// surface was built from. See docs/evaluator-design.md §6.

// Build builds the revolved body rp records. work is the record's free-form
// work counter: an operation that already holds one passes it, so the
// preflight below and every walkOf under it continue that counter rather than
// open a second ceiling on the same record; a re-evaluation passes a fresh one.
func Build(ctx context.Context, env topo.Env, rp Payload, work *freeform.FreeformWork) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := rp.Profile.EvaluatorIntegralsUncheckedContext(ctx, freeform.MomentSecondOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, decaderr.ErrDegenerate)
	}
	// Every reading below integrates the RECORDED region while every face below
	// is built from the SNAPPED one (revolveaxis.Frame's walk), so each
	// integral owes what the snap moved its own integrand's region by
	// (regionSnapAllow). The area's share lands here, before the cap faces read
	// it; the three axis-frame moments' shares land on q, mzr and mrr below,
	// since those are the coordinates their envelopes were proven in. Every one
	// of the four is exactly zero for a profile whose on-axis endpoints already
	// sit on the axis.
	ig.AreaBound = proofbound.AbsSumUpper(ig.AreaBound, rp.Ax.Snap.Area)
	sweep := rp.Sweep()
	dphi := sweep.Value
	if dphi <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, decaderr.ErrDegenerate)
	}
	frame, err := rp.frameCharge()
	if err != nil {
		return nil, err
	}
	q, mzr, mrr := revolvemass.AxisMoments(ig, rp.Ax)
	// The snap's own share of each axis-frame moment, charged where the moment
	// is read rather than back on the plane-local integrals it was composed
	// from: ρ and z are what regionSnapAllow's envelopes were proven against,
	// and a profile far down the axis has a large |z| beside a small ρ, so
	// charging the volume's ∫ρ dA at a frame-origin envelope would inflate it by
	// the whole axial offset. proofbound.BoundedMul and proofbound.BoundedDiv below then carry these
	// into the volume and the centroid through the arithmetic they already run.
	q.Bound = proofbound.AbsSumUpper(q.Bound, rp.Ax.Snap.First)
	mzr.Bound = proofbound.AbsSumUpper(mzr.Bound, rp.Ax.Snap.Mixed)
	mrr.Bound = proofbound.AbsSumUpper(mrr.Bound, rp.Ax.Snap.Second)
	if q.Value <= 0 {
		return nil, fmt.Errorf(`%w: the region has no material off the revolve axis`, decaderr.ErrDegenerate)
	}

	b := rp.Basis()
	// A surface result publishes a sheet, never a solid: solid false is what
	// makes Volume() and Centroid() answer ErrNotSolid through their existing
	// guards, and kind BodySheet is what Kind() reports
	// (docs/surface-design.md §4.3).
	kind := topo.BodySolid
	if rp.SurfaceResult {
		kind = topo.BodySheet
	}
	ref := env.Ref
	body := env.Alloc.NewBody()
	body.Solid = !rp.SurfaceResult
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Kind = kind
	body.Doc = env.Doc

	// Partial sweeps get two planar cap faces; a full revolution has none.
	var capStart, capEnd *topo.Face
	if !rp.Full {
		startFrame, err := revolvemesh.CapFrame(b, rp.Xform, rp.Phi0, true)
		if err != nil {
			return nil, err
		}
		endFrame, err := revolvemesh.CapFrame(b, rp.Xform, rp.Phi1, false)
		if err != nil {
			return nil, err
		}
		// normalBound is the cap plane's own dimensionless tilt: the frame's
		// unit normal rotates about the axis at unit rate in φ, so the angle
		// this cap's own end denotes charges its plane's normal by exactly
		// that end's proven displacement. denoted is the plane the record
		// states, which NormalAt proves the cap's normal against in place of
		// the tag and normalBound, so the axis direction, the frame lift and
		// every placement rounding are charged too; normalBound stays the
		// figure the gates that refuse a departed face read
		// (docs/evaluator-design.md §6).
		//
		// capAdmitAllow is revolvemass.AdmitBandCharge's own term (above): each
		// cap face's LOOP is walked from the axis-snapped profile, while its
		// area here is ig.Area, the Pappus engine's own integral over the
		// UNSNAPPED recorded one. The two agree exactly wherever
		// rp.Ax.RadialAdmitAllow is zero — every axis-aligned fixture — and
		// otherwise this is what keeps the published cap area from claiming a
		// tighter bound than the snap/unsnap mismatch can actually cost it.
		capAdmitAllow := revolvemass.AdmitBandCharge(rp.Ax.RadialAdmitAllow, rp.Ax.AxialExtentUpper)
		capStart = topobuild.NewFace(env.Alloc, topo.Face{
			Area:        ig.Area,
			AreaBound:   proofbound.AbsSumUpper(ig.AreaBound, capAdmitAllow),
			NormalBound: rp.phi0Delta(),
			Denoted:     rp.denotationInput().CapDenotation(rp.Den.Phi0, rp.Phi0, true),
			Surface:     surfacegeom.Plane{Frame: startFrame},
			Origins:     []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:        body,
		})
		capEnd = topobuild.NewFace(env.Alloc, topo.Face{
			Area:        ig.Area,
			AreaBound:   proofbound.AbsSumUpper(ig.AreaBound, capAdmitAllow),
			NormalBound: rp.phi1Delta(),
			Denoted:     rp.denotationInput().CapDenotation(rp.Den.Phi1, rp.Phi1, false),
			Surface:     surfacegeom.Plane{Frame: endFrame},
			Origins:     []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:        body,
		})
	}

	// The section displacement's region charge (docs/surface-intersection-design.md
	// §7.2): the recorded and denoted regions differ by a band of at most
	// section.Band, so the cap area and the three axis-frame moments each move by
	// their integrand's envelope over it. The snap's charges above are about the
	// recorded region against the snapped one and compose beside this one. Zero
	// for every payload no construction displaced, and folded nowhere then.
	section, err := revolveaxis.ChargeOf(rp.Profile, rp.Ax, rp.SectionDelta, work)
	if err != nil {
		return nil, err
	}
	if section.Band > 0 {
		q.Bound = proofbound.AbsSumUpper(q.Bound, section.First())
		mzr.Bound = proofbound.AbsSumUpper(mzr.Bound, section.Second())
		mrr.Bound = proofbound.AbsSumUpper(mrr.Bound, section.Second())
		if !rp.Full {
			capStart.AreaBound = proofbound.AbsSumUpper(capStart.AreaBound, section.Band)
			capEnd.AreaBound = proofbound.AbsSumUpper(capEnd.AreaBound, section.Band)
		}
	}
	// Every charge above bounds the cap's plane-coordinate area; its image
	// under the denoted map L takes L's own stretch last (frameCharge).
	if !rp.Full {
		for _, c := range []*topo.Face{capStart, capEnd} {
			c.AreaBound = frame.AreaOf(proofbound.MeasuredScalar(c.Area, c.AreaBound)).Bound
		}
	}

	sideArea := proofbound.BoundedScalar{}
	loops := append([]sectionrecord.LoopRecord{rp.Profile.Outer}, rp.Profile.Holes...)
	perLoop := make([]revLoopParts, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		parts, err := buildRevolveLoop(ctx, env.Alloc, body, ref, rp, b, li, loop, work, frame)
		if err != nil {
			return nil, err
		}
		perLoop[li] = parts
		sideArea = proofbound.BoundedAdd(sideArea, parts.area)
		if !rp.Full {
			capStart.Loops = append(capStart.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: parts.startCo}))
			capEnd.Loops = append(capEnd.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: parts.endCo}))
		}
	}

	// Shells: a partial sweep's caps connect every loop's walls into one
	// boundary, but a full revolution encloses each profile hole as its own
	// toroidal void — a separate shell, and a void one (evaluator §3). A
	// surface result never claims that void (decision B,
	// docs/surface-design.md §2.2): its hole loop's own closed surface
	// touches the outer one nowhere, so it is a wholly separate piece of the
	// body's boundary and gets its own Lump rather than a second shell of the
	// solid's one (decision A).
	var lumps []*topo.Lump
	if rp.Full {
		shells, err := fullRevolveShellsContext(ctx, env.Alloc, perLoop, rp.SurfaceResult)
		if err != nil {
			return nil, err
		}
		if rp.SurfaceResult {
			lumps = make([]*topo.Lump, len(shells))
			for i, sh := range shells {
				lumps[i] = env.Alloc.NewLump()
				lumps[i].Shells = []*topo.FaceShell{sh}
			}
		} else {
			lump := env.Alloc.NewLump()
			lump.Shells = shells
			lumps = []*topo.Lump{lump}
		}
	} else {
		var faces []*topo.Face
		for _, group := range perLoop {
			faces = append(faces, group.faces...)
		}
		// A surface result omits both caps from the shell (Table W,
		// docs/surface-design.md §4.1-§4.2): capStart/capEnd stay constructed
		// above so the area accumulation below can still read their area
		// fields, but neither joins faces nor gets its loops attached, which
		// is what leaves each rim edge with only its wall face — both
		// cap-plane rims of every wall become free edges with no
		// rim-specific code.
		if !rp.SurfaceResult {
			faces = append(faces, capStart, capEnd)
			if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{capStart, capEnd}); err != nil {
				return nil, err
			}
		}
		if rp.SurfaceResult {
			// topobuild.SheetLumps splits by connectivity (decision A): a holed
			// profile's loops touch nowhere once their caps are omitted, so
			// each becomes its own Lump instead of one shell regardless.
			lumps = topobuild.SheetLumps(env.Alloc, faces)
		} else {
			shell := env.Alloc.NewFaceShell()
			shell.Open = topobuild.ShellIsOpen(faces)
			shell.Faces = faces
			lump := env.Alloc.NewLump()
			lump.Shells = []*topo.FaceShell{shell}
			lumps = []*topo.Lump{lump}
		}
	}
	body.Lumps = lumps
	if err := topobuild.AddBlendRoles(ctx, body, ref, rp.BlendSegs, rp.BlendKind); err != nil {
		return nil, err
	}
	if rp.DraftCircleCaps {
		if err := addDraftCircleCapRoles(ctx, body, ref); err != nil {
			return nil, err
		}
	}
	if rp.DraftCircleShellRoles {
		if err := addDraftCircleShellRoles(ctx, body, ref); err != nil {
			return nil, err
		}
	}

	// Measurements — Pappus with the profile and float-evaluation bounds
	// carried through (docs/evaluator-design.md §6).
	area := sideArea
	if !rp.Full {
		// Both caps' own (area, areaBound) are read back rather than
		// re-derived from ig directly, so the solid's aggregate area takes the
		// SAME admitted-band charge (capAdmitAllow, above) the individual cap
		// Face fields already carry — never a narrower, uncharged bound the
		// two would then disagree with.
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(capStart.Area, capStart.AreaBound))
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(capEnd.Area, capEnd.AreaBound))
		if rp.SurfaceResult {
			// §4.3: a surface result's area is the solid's area minus the two
			// omitted caps'. Each cap's area and bound are read back from the
			// Face fields the construction above stamped, and the
			// subtraction runs here — as the LAST thing that touches area,
			// since revolve carries no per-cap bound re-stamp the way the
			// prism's prismrecord.Build does, so nothing after this point may
			// still widen capStart's or capEnd's own bound out from under a
			// value already composed from them. Collapsing this to sideArea
			// alone would hold the same value but a smaller, uncomposed
			// bound than §4.3's composed sum requires.
			area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capStart.Area, capStart.AreaBound))
			area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capEnd.Area, capEnd.AreaBound))
		}
	}
	volume := proofbound.BoundedMul(q, sweep)
	volume.Bound = proofbound.AbsSumUpper(volume.Bound,
		revolvemass.AdmitVolumeCharge(rp.Ax.RadialAdmitAllow, rp.Ax.AxialExtentUpper))
	// The Pappus volume is the plane-coordinate solid's; L scales it by
	// exactly |det L| (frameCharge).
	volume = frame.VolumeOf(volume)
	body.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: measurement.ExactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: measurement.ExactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}

	coordUpper, err := momentinput.CoordinateEnvelope(rp.Profile, work, nil)
	if err != nil {
		return nil, err
	}
	centroidValue, centroidBound := revolvemass.Centroid(revolvemass.CentroidInput{
		Basis: b, Frame: rp.Frame, Placement: rp.Xform, Axis: rp.Ax,
		Full: rp.Full, Phi0: rp.Phi0, Phi1: rp.Phi1, DenPhi0: rp.Den.Phi0, DenPhi1: rp.Den.Phi1,
		Q: q, MZR: mzr, MRR: mrr, Sweep: sweep,
		CoordUpper: coordUpper, SectionDelta: rp.SectionDelta,
		RadialAdmit: rp.Ax.RadialAdmitAllow, AxialExtentUpper: rp.Ax.AxialExtentUpper,
	})
	body.Centroid = measurement.VecMeasurement{
		Value:     centroidValue,
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	bounds, err := Bounds(ctx, rp, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = rp
	return body, nil
}

// revLoopParts is what one recorded loop contributes to the revolved body.
type revLoopParts struct {
	faces   []*topo.Face
	startCo []topo.CoEdge            // the loop's start-cap coedges, walk order (partial only)
	endCo   []topo.CoEdge            // the loop's end-cap coedges, walk order (partial only)
	area    proofbound.BoundedScalar // the loop's side-face area
	// runs[i] is faces[i]'s run: the off-axis stretch of the loop between two
	// on-axis walks that holds it. A loop with fewer than two on-axis walks is
	// one run. outerRun is the run holding the loop's on-axis junction of least
	// axial coordinate.
	runs     []int
	outerRun int
}

// junctionRadiusInterval encloses a junction's radial coordinate ρ as
// [ρ−ρBound, ρ+ρBound], the rational-interval twin of the proven float bound
// revolveaxis.Frame.walk already composed into startV/startVBound: ρ is a walk's own
// startV (a junction sits at walk i's start), and ρBound is startVBound, the
// PROVEN error between it and the axis-re-expressed value the walk's true
// (unrounded) axis direction and anchor would give. ok is false wherever
// either cannot be stated as an exact rational — a non-finite ρBound is the
// refusal shape every consumer below already turns into its own envelope.
func junctionRadiusInterval(rho, rhoBound float64) (proofbound.RatInterval, bool) {
	r, b := proofarith.FloatRat(rho), proofarith.FloatRat(math.Abs(rhoBound))
	if r == nil || b == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(r), b), true
}

// revJunction is one junction between consecutive walks: the shared point in
// axis coordinates and what it sweeps to — nothing on the axis, a full
// latitude circle for a full revolution, an arc between the two caps for a
// partial sweep.
type revJunction struct {
	z, rho float64
	onAxis bool
	v0, v1 *topo.Vertex // partial: start-/end-cap vertices (one shared on the axis)
	arc    *topo.Edge   // partial: the swept junction arc; nil on the axis
	lat    *topo.Edge   // full: the latitude circle; nil on the axis
}

// buildRevolveLoop builds one loop's side faces with shared vertices and
// edges, returning the faces, the two caps' coedges in walk order, and the
// loop's side area.
func buildRevolveLoop(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer, rp Payload, b revolvemesh.RevolveBasis, li int, loop sectionrecord.LoopRecord, work *freeform.FreeformWork, frame massmoment.MapCharge) (revLoopParts, error) {
	resolved, err := revolveaxis.ResolveLoopWithFreeform(ctx, loop, work, "the revolve wall build",
		rp.ChargedWalk, rp.Ax.SnapTol, rp.Ax.RadialLower)
	if err != nil {
		return revLoopParts{}, err
	}
	walks, kinds, singleClosed := resolved.Walks, resolved.Kinds, resolved.SingleClosed
	n := len(walks)
	sweep := rp.Sweep()
	dphi := sweep.Value

	// Junction vertices and swept edges: junction i sits at walk i's start
	// (== walk i−1's end). A single whole closed curve has none; a junction
	// on the axis sweeps to a single point — no edge, and for a partial
	// sweep one vertex shared by both caps. Convexity comes from the 2D
	// turn, exactly as the prism's vertical edges: a positive cross of the
	// incoming and outgoing tangents is a convex edge on the outer loop and
	// works out identically for hole loops walked clockwise.
	var js []revJunction
	if !singleClosed {
		js = make([]revJunction, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			j := revJunction{z: w.StartU, rho: w.StartV, onAxis: w.StartV == 0}
			// The recorded point the junction denotes: walk i's start is its
			// first recorded segment's own start (revolvesampling.MeridianJunctions,
			// tessellate_revolve.go, reads the same one), bounded against the
			// previous walk's denoted end too.
			prev := walks[(i+n-1)%n]
			at := rp.denotedPoint(revolveJunctionStart(resolved, prev, w))
			turn := prev.TanOutU*w.TanInV - prev.TanOutV*w.TanInU
			center, jAxis, jRadius := rp.JunctionCircle(b, j.z, j.rho)
			switch {
			case rp.Full && !j.onAxis:
				seam := rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end0())
				latitudeLength := 2 * math.Pi * j.rho
				latitudeBound := proofbound.ConservativeValueError(latitudeLength, proofbound.ProductUpper(w.AxisRadiusUpper, proofbound.TwoPiUpper()))
				if rhoEnc, ok := junctionRadiusInterval(j.rho, w.StartVBound); ok {
					enc := proofbound.IntervalMul(proofbound.TwoPiInterval(), rhoEnc)
					latitudeBound = math.Min(latitudeBound, proofbound.IntervalFloatError(enc, latitudeLength))
				}
				j.lat = topobuild.NewEdge(alloc, topo.Edge{
					Convex:      turn > 0,
					Length:      latitudeLength,
					LengthBound: frame.LengthBound(latitudeLength, latitudeBound),
					// The CURVE half of the shared-denotation certificate
					// (denotation.go): this junction latitude circle is
					// shared, by construction, between side faces i-1 and i
					// of THIS build alone (fullRevLoops below), exactly the
					// role a prism's own rim edge plays for its two caps.
					Denot: body.Doc.MintCurve(),
					Curve: surfacegeom.Circle3{Center: center, Axis: jAxis, Radius: jRadius},
					Start: seam,
					End:   seam,
				})
			case !rp.Full:
				if j.onAxis {
					j.v0 = rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end0(), rp.end1())
					j.v1 = j.v0
				} else {
					j.v0 = rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end0())
					j.v1 = rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end1())
					arcLength := j.rho * dphi
					dphiUpper := proofbound.AbsSumUpper(math.Abs(dphi), sweep.Bound)
					arcBound := proofbound.ConservativeValueError(arcLength, proofbound.ProductUpper(w.AxisRadiusUpper, dphiUpper))
					if rhoEnc, ok := junctionRadiusInterval(j.rho, w.StartVBound); ok {
						if widthEnc, ok := rp.Den.WidthInterval(); ok {
							enc := proofbound.IntervalMul(rhoEnc, widthEnc)
							arcBound = math.Min(arcBound, proofbound.IntervalFloatError(enc, arcLength))
						}
					}
					j.arc = topobuild.NewEdge(alloc, topo.Edge{
						Convex:      turn > 0,
						Length:      arcLength,
						LengthBound: frame.LengthBound(arcLength, arcBound),
						// The CURVE half of the shared-denotation certificate
						// (denotation.go): this junction arc is shared, by
						// construction, between side faces i-1 and i of THIS
						// build alone.
						Denot: body.Doc.MintCurve(),
						Curve: surfacegeom.Arc3{Center: center, Axis: jAxis, Radius: jRadius},
						Start: j.v0,
						End:   j.v1,
					})
				}
			}
			for _, je := range []*topo.Edge{j.lat, j.arc} {
				if je != nil {
					je.CurveBound, je.CurveBounded = rp.latitudeCurveBound(at, center, jAxis, jRadius.Base())
				}
			}
			js[i] = j
		}
	}

	// Cap edges (partial sweeps only): each walk's copy in each cap plane.
	// A line lying on the axis does not move when swept, so its two copies
	// coincide: ONE edge, shared by both caps — that is how the caps of a
	// solid wedge meet along the axis. Its dihedral is the sweep angle
	// itself.
	holeLoop := li != 0
	var cap0, cap1 []*topo.Edge
	if !rp.Full {
		cap0 = make([]*topo.Edge, n)
		cap1 = make([]*topo.Edge, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			if kinds[i] == revolveaxis.WallAxis {
				// This edge is appended only to cap0/cap1 below, never to any
				// wall face's own loop — a LineSeg on the axis emits no wall
				// face at all (evaluator §6). A solid's attached caps are
				// what give it its two faces; a surface result never attaches
				// them, so nothing else references this edge and Body.Edges()
				// never reaches it — Table W row 3's on-axis-edge omission
				// delivered by omission, not a special case to "fix" here.
				shared := topobuild.NewEdge(alloc, topo.Edge{
					Convex:      dphi < math.Pi,
					Length:      w.Length,
					LengthBound: frame.LengthBound(w.Length, w.LengthBound),
					Curve:       surfacegeom.Line3{},
					Start:       js[i].v0,
					End:         js[(i+1)%n].v0,
				})
				cap0[i], cap1[i] = shared, shared
				continue
			}
			var vs0, ve0, vs1, ve1 *topo.Vertex
			if !singleClosed {
				vs0, ve0 = js[i].v0, js[(i+1)%n].v0
				vs1, ve1 = js[i].v1, js[(i+1)%n].v1
			}
			// A whole closed walk's seam vertex denotes its one recorded
			// segment's own start; every other walk's cap edge takes the
			// junction vertices above.
			seam := rp.denotedPoint(walkStart(resolved.Segs[w.Segs[0]], resolved.Plane[w.Segs[0]]))
			plane := segPlaneWalks(resolved, w.Segs)
			freeformConvex := false
			if w.Kind == survey2d.WalkFreeform {
				freeformConvex, err = boundarywalk.RimConvexity(ctx, w, holeLoop, work)
				if err != nil {
					return revLoopParts{}, err
				}
			}
			cap0[i] = rp.capEdge(alloc, b, w.SegmentWalk, plane, singleClosed, vs0, ve0, seam, rp.end0(), holeLoop, freeformConvex, frame)
			cap1[i] = rp.capEdge(alloc, b, w.SegmentWalk, plane, singleClosed, vs1, ve1, seam, rp.end1(), holeLoop, freeformConvex, frame)
		}
	}

	parts := revLoopParts{}
	axisWalks := 0
	for _, k := range kinds {
		if k == revolveaxis.WallAxis {
			axisWalks++
		}
	}
	// A full turn sweeps each run into its own closed surface: an on-axis
	// junction sweeps a point, never an edge, so two runs share no edge. Of
	// those surfaces the outer one encloses every other, so its two axis ends
	// bracket theirs (fullRevolveShellsContext).
	seen, leastZ := 0, math.Inf(1)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return revLoopParts{}, err
		}
		run := 0
		if axisWalks > 1 {
			// The faces after the last on-axis walk close the loop onto the
			// faces before the first, so they share its run.
			run = seen % axisWalks
		}
		if kinds[i] == revolveaxis.WallAxis {
			seen++
		} else {
			for _, end := range [...][2]float64{{w.StartU, w.StartV}, {w.EndU, w.EndV}} {
				if end[1] == 0 && end[0] < leastZ {
					leastZ, parts.outerRun = end[0], run
				}
			}
		}
		if kinds[i] == revolveaxis.WallAxis {
			if !rp.Full {
				parts.startCo = append(parts.startCo, topo.CoEdge{Edge: cap0[i], Forward: true})
				parts.endCo = append(parts.endCo, topo.CoEdge{Edge: cap1[i], Forward: true})
			}
			continue
		}
		surf, reversed, err := rp.WallSurface(b, w.SegmentWalk, kinds[i])
		if err != nil {
			return revLoopParts{}, err
		}
		origins, err := prismrecord.SideOrigins(ctx, ref, li, w.Segs)
		if err != nil {
			return revLoopParts{}, err
		}
		if li < len(rp.SourceSideOrigins) {
			for _, si := range w.Segs {
				if si < len(rp.SourceSideOrigins[li]) {
					origins = append(origins, rp.SourceSideOrigins[li][si]...)
				}
			}
		}
		segs := make([]sectionrecord.CurveSegment, len(w.Segs))
		for j, si := range w.Segs {
			segs[j] = loop.Segments[si]
		}
		faceArea := frame.AreaOf(proofbound.BoundedMul(revolvemass.ChargedWallAxisMoment(
			w.SegmentWalk, kinds[i], segs, rp.Ax, rp.SectionWhole, rp.SectionDelta), sweep))
		face := topobuild.NewFace(alloc, topo.Face{
			Area:      faceArea.Value,
			AreaBound: faceArea.Bound,
			Reversed:  reversed,
			Denoted:   rp.denotationInput().WallDenotation(w, kinds[i], resolved.Plane),
			Surface:   surf,
			Origins:   origins,
			Body:      body,
		})
		switch {
		case rp.Full && singleClosed:
			// A whole closed generator swept a full turn bounds a closed
			// surface — a torus needs no boundary loop at all.
		case rp.Full:
			face.Loops = fullRevLoops(alloc, js[i], js[(i+1)%n], kinds[i])
		case singleClosed:
			// A closed generator's partial sweep is bounded by its two cap
			// copies — two loops, one whole closed edge each, mirroring the
			// prism's whole-circle two-loop discipline.
			face.Loops = []*topo.Loop{
				topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: cap0[i], Forward: true}}}),
				topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: cap1[i], Forward: false}}}),
			}
		default:
			co := []topo.CoEdge{{Edge: cap0[i], Forward: true}}
			if a := js[(i+1)%n].arc; a != nil {
				co = append(co, topo.CoEdge{Edge: a, Forward: true})
			}
			co = append(co, topo.CoEdge{Edge: cap1[i], Forward: false})
			if a := js[i].arc; a != nil {
				co = append(co, topo.CoEdge{Edge: a, Forward: false})
			}
			face.Loops = []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: co})}
		}
		if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{face}); err != nil {
			return revLoopParts{}, err
		}
		parts.faces = append(parts.faces, face)
		parts.runs = append(parts.runs, run)
		parts.area = proofbound.BoundedAdd(parts.area, faceArea)
		if !rp.Full {
			parts.startCo = append(parts.startCo, topo.CoEdge{Edge: cap0[i], Forward: true})
			parts.endCo = append(parts.endCo, topo.CoEdge{Edge: cap1[i], Forward: true})
		}
	}
	return parts, nil
}

// fullRevolveShellsContext builds one shell per non-empty run of each loop
// group. A loop meeting the axis along k ≥ 2 walks has k runs, and the full
// turn sweeps each into its own closed surface (a hollow cylinder's meridian
// sweeps an outer skin and a cavity wall); every other loop is one run. sheet
// is rp.SurfaceResult: a full revolution's wall set already closes on itself,
// so every shell built here is closed regardless of kind, but topobuild.ShellIsOpen
// still runs rather than assuming it — the same discipline every other shell
// in this build follows (docs/surface-design.md §2.2). On a solid, a hole
// loop's run bounds its own toroidal cavity (evaluator §6), and so does every
// run of the outer loop but its outer one: the outer surface encloses the
// others, so its two axis ends bracket every other on-axis junction of the
// loop. A sheet bounds no cavity at all (decision B, §2.2), so void is always
// false there.
func fullRevolveShellsContext(ctx context.Context, alloc topo.Alloc, perLoop []revLoopParts, sheet bool) ([]*topo.FaceShell, error) {
	var shells []*topo.FaceShell
	for li, parts := range perLoop {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		// Runs in order of first appearance, the outer run first.
		order := []int{parts.outerRun}
		groups := map[int][]*topo.Face{}
		for i, f := range parts.faces {
			r := parts.runs[i]
			if _, ok := groups[r]; !ok && r != parts.outerRun {
				order = append(order, r)
			}
			groups[r] = append(groups[r], f)
		}
		for _, r := range order {
			group := groups[r]
			if len(group) == 0 {
				continue
			}
			void := !sheet && (li != 0 || r != parts.outerRun)
			shell := alloc.NewFaceShell()
			shell.Open = topobuild.ShellIsOpen(group)
			shell.Void = void
			shell.Faces = group
			shells = append(shells, shell)
		}
	}
	return shells, nil
}

// fullRevLoops assembles a full-revolution side face's boundary loops from
// its junctions' latitude circles — none on the axis (a pole or an apex
// bounds nothing), and for a planar annulus the inner circle is genuinely a
// hole; every other face's circles are both outer, like the prism's whole
// cylinder.
func fullRevLoops(alloc topo.Alloc, j0, j1 revJunction, kind revolveaxis.WallKind) []*topo.Loop {
	outer0, outer1 := true, true
	if kind == revolveaxis.WallPlane && !j0.onAxis && !j1.onAxis {
		if j0.rho < j1.rho {
			outer0 = false
		} else {
			outer1 = false
		}
	}
	var loops []*topo.Loop
	if j0.lat != nil {
		loops = append(loops, topobuild.NewLoop(alloc, topo.Loop{Outer: outer0, CoEdges: []topo.CoEdge{{Edge: j0.lat, Forward: true}}}))
	}
	if j1.lat != nil {
		loops = append(loops, topobuild.NewLoop(alloc, topo.Loop{Outer: outer1, CoEdges: []topo.CoEdge{{Edge: j1.lat, Forward: false}}}))
	}
	if len(loops) == 2 && !loops[0].Outer {
		loops[0], loops[1] = loops[1], loops[0]
	}
	return loops
}

// capEdge is one boundary walk's copy in the cap plane at sweep angle φ: a
// straight walk stays a line, a circular one an arc — or a whole circle
// with a seam vertex, closing on itself. The cap edge sits between the cap
// and the side face it bounds; the material across it is a quarter-turn wedge
// everywhere, so the material angle decides nothing — a rim reports the
// WALKED-BOUNDARY convexity instead (topology.go, Edge.IsConvex), which is the
// side the wall's material lies on. A CIRCULAR walk decides that by its own
// sense (a clockwise arc keeps the
// material outside the sphere/torus it sweeps, exactly as wallSurface reads
// it, so its cap edge is concave — a hole's arc and a concave bite in the
// outer boundary alike), while a STRAIGHT walk has no sense of its own and
// takes the loop's: outer convex, hole concave. end is THIS cap's end of the
// sweep (rp.end0() or rp.end1()), and seam the recorded point a closed walk's
// own seam vertex denotes; that vertex is bounded the same way every other cap
// vertex is (sweptVertex; docs/evaluator-design.md §6). An open walk reads
// neither.
func (rp Payload) capEdge(alloc topo.Alloc, b revolvemesh.RevolveBasis, w survey2d.SegmentWalk, plane []survey2d.SegmentWalk, closed bool, vs, ve *topo.Vertex, seam sweptPoint, end sweptEnd, holeLoop, freeformConvex bool, frame massmoment.MapCharge) *topo.Edge {
	convex := !holeLoop
	if w.IsCircular() {
		convex = w.Th0 < w.Th1
	}
	if w.Kind == survey2d.WalkFreeform {
		convex = freeformConvex
	}
	e := topobuild.NewEdge(alloc, topo.Edge{Convex: convex, Length: w.Length, LengthBound: frame.LengthBound(w.Length, w.LengthBound)})
	if w.Kind == survey2d.WalkFreeform {
		e.Curve = surfacegeom.NURBSCurve{}
		if closed {
			v := rp.sweptVertex(alloc, b, w.StartU, w.StartV, seam, denotation.CurveToken{}, end)
			e.Start, e.End = v, v
		} else {
			e.Start, e.End = vs, ve
		}
		return e
	}
	if !w.IsCircular() {
		e.Curve = surfacegeom.Line3{}
		e.Start, e.End = vs, ve
		return e
	}
	// The cap plane at φ is the profile plane rotated about the axis, so
	// its normal is the rotated sweep-velocity direction; the walk's own
	// range order is the arc's CCW sense about it, inverted once by a
	// reflected placement.
	sin, cos := math.Sincos(end.phi)
	normal := b.E0.Scale(-sin).Add(b.E1.Scale(cos))
	sign := 1.0
	if w.Th1 < w.Th0 {
		sign = -1
	}
	if rp.Reflected() {
		sign = -sign
	}
	axis := rp.Xform.ApplyDir(normal).Scale(sign)
	center := rp.point(b, w.CU, w.CV, end.phi)
	radius := units.Millimeters(w.Radius)
	e.CurveBound, e.CurveBounded = rp.denotationInput().CapArcCurveBound(plane, end.den, end.phi, center, axis, w.Radius)
	if closed {
		v := rp.sweptVertex(alloc, b, w.StartU, w.StartV, seam, denotation.CurveToken{}, end)
		e.Curve = surfacegeom.Circle3{Center: center, Axis: axis, Radius: radius}
		e.Start, e.End = v, v
		return e
	}
	e.Curve = surfacegeom.Arc3{Center: center, Axis: axis, Radius: radius}
	e.Start, e.End = vs, ve
	return e
}

// This section is chain Revolve's own build (docs/surface-design.md §13.4):
// the open chain's counterpart of Build/buildRevolveLoop
// above, reusing rp.WallSurface, rp.capEdge, prismrecord.SideOrigins and
// revolvemass.WallAxisMoment unchanged. It differs only in TOPOLOGY: n segments place
// n+1 junctions with no wraparound, and neither cap face nor cap coedge
// collection is ever built — a chain mints no cap.

// BuildChain builds the shell body: one swept wall per recorded
// segment (Table G rows 2-3), never a cap and never a wraparound junction.
// work is the record's ONE free-form work counter (docs/spline-design.md
// §5.2): the caller opens it once and this build and the final bounds
// reading both spend from it.
func BuildChain(ctx context.Context, env topo.Env, rp ChainPayload, work *freeform.FreeformWork) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(rp.Chains) == 0 {
		return nil, fmt.Errorf(`%w: a chain-revolve payload holds no walk`, decaderr.ErrDegenerate)
	}
	rev := rp.Revolve()
	sweep := rev.Sweep()
	if sweep.Value <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, decaderr.ErrDegenerate)
	}

	frame, err := rev.frameCharge()
	if err != nil {
		return nil, err
	}
	ref := env.Ref
	body := env.Alloc.NewBody()
	body.Solid = false
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	b := rev.Basis()
	var allFaces []*topo.Face
	area := proofbound.BoundedScalar{}
	for ci := range rp.Chains {
		// Each walk builds from its OWN single-walk view, so
		// prismrecord.SideOrigins's segment indices and revolvemass.WallAxisMoment's own
		// rp.Profile.Outer read reach exactly that walk's segments, and ci
		// feeds the face role so two walks never collide on one role string
		// (docs/surface-intersection-design.md §3.4).
		view := rp.WalkView(ci)
		resolved, err := revolveaxis.ResolveChain(ctx, rp.Chains[ci], work, "the revolve wall build",
			view.ChargedWalk, view.Ax.SnapTol)
		if err != nil {
			return nil, err
		}
		if len(resolved.Walks) == 0 {
			return nil, fmt.Errorf(`%w: a recorded chain holds no segments`, decaderr.ErrDegenerate)
		}
		if err := revolveaxis.RequireChainAxisIncidence(resolved); err != nil {
			return nil, err
		}
		faces, walkArea, err := buildChainRevolveWalls(ctx, env.Alloc, body, ref, view, ci, b, resolved, frame)
		if err != nil {
			return nil, err
		}
		allFaces = append(allFaces, faces...)
		area = proofbound.BoundedAdd(area, walkArea)
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, allFaces)
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: measurement.ExactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}
	// volume and centroid stay at their zero value: a chain bounds no region
	// (docs/surface-design.md §13.3), so neither is ever integrated here, and
	// neither is reachable through Body.Volume/Body.Centroid while solid is
	// false (§8) — exactly as chainrecord.Build leaves them.
	bounds, err := Bounds(ctx, rev, work)
	if err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = rp
	return body, nil
}

// buildChainRevolveWalls builds an open chain's whole swept-wall set with
// shared vertices and edges: one wall per recorded segment, never a cap and
// never a wraparound junction. n segments place n+1 junctions, so the
// chain's two FREE ends (junction 0 and junction n) each carry a swept edge
// (an arc, or a full latitude circle under a full revolution) touched by
// exactly one wall — Free() resolves them (Edge.IsFree, topology.go) —
// while every INTERIOR junction (1..n-1) shares its swept edge between the
// two walls it joins, exactly as buildRevolveLoop's own junctions do for a
// closed loop. A wall's own copy of its walk at Phi0/Phi1 (capEdge) is never
// attached to a cap face here — a chain mints none — so it stays free
// regardless of position (docs/surface-design.md §13.4). It returns the
// faces and the walk's own total wall area, folded through proofbound.BoundedAdd.
func buildChainRevolveWalls(ctx context.Context, alloc topo.Alloc, body *topo.Body, ref provenance.Producer, rp Payload, loopIdx int, b revolvemesh.RevolveBasis, resolved revolveaxis.ResolvedWalks, frame massmoment.MapCharge) ([]*topo.Face, proofbound.BoundedScalar, error) {
	walks, kinds := resolved.Walks, resolved.Kinds
	n := len(walks)
	sweep := rp.Sweep()
	dphi := sweep.Value
	sweepSign := 1.0
	if rp.Reflected() {
		sweepSign = -1
	}
	wDir := rp.Xform.ApplyDir(b.W)

	// junctionSource reads junction i's own (z, rho), the walk whose end it
	// belongs to, and the recorded point it denotes: junction i sits at walk
	// i's start for i < n, and at the LAST walk's own end for i == n — the
	// chain's own two free ends. A coalesced walk starts at its first recorded
	// segment's start and ends at its last one's end.
	junctionSource := func(i int) (z, rho, rhoBound, axisRadiusUpper float64, at sweptPoint) {
		switch i {
		case n:
			w := walks[n-1]
			last := w.Segs[len(w.Segs)-1]
			return w.EndU, w.EndV, w.EndVBound, w.AxisRadiusUpper, rp.denotedPoint(walkEnd(resolved.Segs[last], resolved.Plane[last]))
		case 0:
			w := walks[0]
			return w.StartU, w.StartV, w.StartVBound, w.AxisRadiusUpper, rp.denotedPoint(walkStart(resolved.Segs[w.Segs[0]], resolved.Plane[w.Segs[0]]))
		}
		w := walks[i]
		return w.StartU, w.StartV, w.StartVBound, w.AxisRadiusUpper, rp.denotedPoint(revolveJunctionStart(resolved, walks[i-1], w))
	}

	js := make([]revJunction, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		z, rho, rhoBound, axisRadiusUpper, at := junctionSource(i)
		j := revJunction{z: z, rho: rho, onAxis: rho == 0}
		var turn float64
		if i > 0 && i < n {
			prev, w := walks[i-1], walks[i]
			turn = prev.TanOutU*w.TanInV - prev.TanOutV*w.TanInU
		}
		center := rp.point(b, j.z, 0, 0)
		switch {
		case rp.Full && !j.onAxis:
			seam := rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end0())
			latitudeLength := 2 * math.Pi * j.rho
			latitudeBound := proofbound.ConservativeValueError(latitudeLength, proofbound.ProductUpper(axisRadiusUpper, proofbound.TwoPiUpper()))
			if rhoEnc, ok := junctionRadiusInterval(j.rho, rhoBound); ok {
				enc := proofbound.IntervalMul(proofbound.TwoPiInterval(), rhoEnc)
				latitudeBound = math.Min(latitudeBound, proofbound.IntervalFloatError(enc, latitudeLength))
			}
			j.lat = topobuild.NewEdge(alloc, topo.Edge{
				Convex:      turn > 0,
				Length:      latitudeLength,
				LengthBound: frame.LengthBound(latitudeLength, latitudeBound),
				Denot:       body.Doc.MintCurve(),
				Curve:       surfacegeom.Circle3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
				Start:       seam,
				End:         seam,
			})
		case !rp.Full:
			if j.onAxis {
				j.v0 = rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end0(), rp.end1())
				j.v1 = j.v0
			} else {
				j.v0 = rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end0())
				j.v1 = rp.sweptVertex(alloc, b, j.z, j.rho, at, body.Doc.MintCurve(), rp.end1())
				arcLength := j.rho * dphi
				dphiUpper := proofbound.AbsSumUpper(math.Abs(dphi), sweep.Bound)
				arcBound := proofbound.ConservativeValueError(arcLength, proofbound.ProductUpper(axisRadiusUpper, dphiUpper))
				if rhoEnc, ok := junctionRadiusInterval(j.rho, rhoBound); ok {
					if widthEnc, ok := rp.Den.WidthInterval(); ok {
						enc := proofbound.IntervalMul(rhoEnc, widthEnc)
						arcBound = math.Min(arcBound, proofbound.IntervalFloatError(enc, arcLength))
					}
				}
				j.arc = topobuild.NewEdge(alloc, topo.Edge{
					Convex:      turn > 0,
					Length:      arcLength,
					LengthBound: frame.LengthBound(arcLength, arcBound),
					Denot:       body.Doc.MintCurve(),
					Curve:       surfacegeom.Arc3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
					Start:       j.v0,
					End:         j.v1,
				})
			}
		}
		for _, je := range []*topo.Edge{j.lat, j.arc} {
			if je != nil {
				je.CurveBound, je.CurveBounded = rp.latitudeCurveBound(at, center, wDir.Scale(sweepSign), j.rho)
			}
		}
		js[i] = j
	}

	faces := make([]*topo.Face, 0, n)
	total := proofbound.BoundedScalar{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		if kinds[i] == revolveaxis.WallAxis {
			// A LineSeg lying exactly on the axis sweeps no face — the same
			// rule a profile-fed revolve's own revolveaxis.WallAxis segment follows
			// (docs/evaluator-design.md §6) — and, with no cap to bridge the
			// gap either side of it, a chain mints nothing at all for it.
			continue
		}
		surf, reversed, err := rp.WallSurface(b, w.SegmentWalk, kinds[i])
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		origins, err := prismrecord.SideOrigins(ctx, ref, loopIdx, w.Segs)
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		segs := make([]sectionrecord.CurveSegment, len(w.Segs))
		for oi, si := range w.Segs {
			segs[oi] = rp.Profile.Outer.Segments[si]
		}
		faceArea := frame.AreaOf(proofbound.BoundedMul(revolvemass.ChargedWallAxisMoment(
			w.SegmentWalk, kinds[i], segs, rp.Ax, rp.SectionWhole, rp.SectionDelta), sweep))
		face := topobuild.NewFace(alloc, topo.Face{
			Area:      faceArea.Value,
			AreaBound: faceArea.Bound,
			Reversed:  reversed,
			Denoted:   rp.denotationInput().WallDenotation(w, kinds[i], resolved.Plane),
			Surface:   surf,
			Origins:   origins,
			Body:      body,
		})
		if rp.Full {
			face.Loops = fullRevLoops(alloc, js[i], js[i+1], kinds[i])
		} else {
			// holeLoop is always false: a chain has no hole, and its whole
			// walk takes the loop-0 (outer) convention
			// (docs/surface-design.md §13.4).
			plane := segPlaneWalks(resolved, w.Segs)
			cap0 := rp.capEdge(alloc, b, w.SegmentWalk, plane, false, js[i].v0, js[i+1].v0, sweptPoint{}, rp.end0(), false, false, frame)
			cap1 := rp.capEdge(alloc, b, w.SegmentWalk, plane, false, js[i].v1, js[i+1].v1, sweptPoint{}, rp.end1(), false, false, frame)
			co := []topo.CoEdge{{Edge: cap0, Forward: true}}
			if a := js[i+1].arc; a != nil {
				co = append(co, topo.CoEdge{Edge: a, Forward: true})
			}
			co = append(co, topo.CoEdge{Edge: cap1, Forward: false})
			if a := js[i].arc; a != nil {
				co = append(co, topo.CoEdge{Edge: a, Forward: false})
			}
			face.Loops = []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: co})}
		}
		if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{face}); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		faces = append(faces, face)
		total = proofbound.BoundedAdd(total, faceArea)
	}
	return faces, total, nil
}

// segPlaneWalks reads the plane-local walks of the recorded segments segs
// indexes.
func segPlaneWalks(resolved revolveaxis.ResolvedWalks, segs []int) []survey2d.SegmentWalk {
	out := make([]survey2d.SegmentWalk, len(segs))
	for i, si := range segs {
		out[i] = resolved.Plane[si]
	}
	return out
}

// A full revolution normally has no cap role: its meridian walls are all
// side faces. Both tapered-circle rim rewrites have two planar disks.
// Preserve their axial cap names so face and rim selectors still work after
// the draft's rebuild and after placement.
func addDraftCircleCapRoles(ctx context.Context, body *topo.Body, ref provenance.Producer) error {
	roles := map[string]string{"side(0,1)": provenance.RoleCapEnd, "side(0,4)": provenance.RoleCapStart}
	seen := map[string]bool{}
	for _, face := range topobuild.Faces(body) {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, origin := range face.Origins {
			capRole, ok := roles[origin.Role]
			if !ok || origin.Producer != ref {
				continue
			}
			if seen[capRole] || face.Surface.Kind() != surfacegeom.KindPlane {
				return fmt.Errorf(`%w: the tapered circular rim has no unique planar %s`, decaderr.ErrUnsupported, capRole)
			}
			face.Origins = append(face.Origins, provenance.Ref{Producer: ref, Role: capRole})
			seen[capRole] = true
		}
	}
	if !seen[provenance.RoleCapStart] || !seen[provenance.RoleCapEnd] {
		return fmt.Errorf(`%w: the tapered circular rim lost a planar cap`, decaderr.ErrUnsupported)
	}
	return nil
}

// addDraftCircleShellRoles gives the revolved meridian the public shell roles
// of a cup. The original side roles stay for the revolve's own readers.
func addDraftCircleShellRoles(ctx context.Context, body *topo.Body, ref provenance.Producer) error {
	roles := map[string]struct {
		name string
		kind surfacegeom.SurfaceKind
	}{
		"side(0,1)": {cupwall.RoleShellCap, surfacegeom.KindPlane},
		"side(0,2)": {"shellSide(0,0)", surfacegeom.KindCone},
		"side(0,3)": {"rim(0)", surfacegeom.KindPlane},
		"side(0,5)": {provenance.RoleCapStart, surfacegeom.KindPlane},
	}
	seen := map[string]struct{}{}
	for _, face := range topobuild.Faces(body) {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, source := range face.Origins {
			role, ok := roles[source.Role]
			if !ok || source.Producer != ref {
				continue
			}
			if face.Surface.Kind() != role.kind {
				return fmt.Errorf(`%w: the tapered circular shell's %s has the wrong surface`,
					decaderr.ErrUnsupported, role.name)
			}
			if _, exists := seen[role.name]; exists {
				return fmt.Errorf(`%w: the tapered circular shell repeats %s`, decaderr.ErrUnsupported, role.name)
			}
			face.Origins = append(face.Origins, provenance.Ref{Producer: ref, Role: role.name})
			seen[role.name] = struct{}{}
		}
	}
	if len(seen) != len(roles) {
		return fmt.Errorf(`%w: the tapered circular shell lost a cap, rim or cavity wall`, decaderr.ErrUnsupported)
	}
	return nil
}
