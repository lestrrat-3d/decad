package loftrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/loft-design.md PR 1a: the evaluator half of Loft — the
// Payload, Table S gates S1-S5, S7's STRUCTURAL arm and S13's
// coordinate-range gate (S9-S11 are the public entry point's job,
// docs/loft-design.md §2/§4), the wiring of the already-landed §6 audit
// (internal/loftmesh/loft_audit.go) and §8 mass kernel (loft_moments.go), and the four
// measurements. Document.Loft is PR 1b.
//
// Sibling files carry the construction Build drives, each with its own doc
// comment: loft_pairing.go decides Table P's correspondence,
// loft_stations.go places the chord stations and proves their departure, and
// topology.go assembles the flat-triangle solid (§5) and its topology.
//
// Payload.Placed and its Delta field are §12 PR 2a (Table D, D7): a
// placement re-lifts both records under the composed motion and re-runs
// §5-§8 from scratch — every record-only Table S gate (S1-S8) and S13's
// coordinate-range gate, plus the placement-only S12, while S9-S11 and S4's
// arity half judge the original
// call's own arguments and never re-run (§4) — so Delta is the ONE new term
// this PR adds, composed into every vertex, edge length, face area and body
// measurement §8 already derives.

// Build builds the lofted body from the payload's own records
// (docs/loft-design.md §5-§8): pairing, assembly, the §6 audit, topology, and
// the four measurements — all four published at build, never staged (§12).
// budget is shared with the rest of the pre-commit cancellation path exactly
// as modify §5's audits already share one; the caller (Loft, PR 1b)
// mints it once for the whole build.
//
// work0/work1 are the per-profile free-form work counters (spline design
// §5.2): the R7 ceiling is one record's across a whole OPERATION, and
// Loft also runs falsifyRecordedArea on both records before Build
// is called, so those counters — not two fresh ones minted here — must be
// the ones every walkOf call site in this build spends against, and the ones
// a same-kind free-form pair's station generator charges
// (docs/loft-design.md §5.1), so a build never opens a second ceiling per
// record.
func Build(ctx context.Context, env topo.Env, pl Payload, budget *proofbound.WorkBudget, work0, work1 *freeform.FreeformWork) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := env.Ref
	pl.RaiseReconstructionLimit(work0, work1)

	offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(pl.Profile0, pl.Profile1, pl.Plane0, pl.Plane1, pl.Alignment, pl.RecordArea, work0, work1)
	if err != nil {
		return nil, err
	}

	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftmesh.PairRecords(pl.Profile0, pl.Profile1, offsets, walks0, walks1, target, work0, work1)
	if err != nil {
		return nil, err
	}
	if pl.ConstructionDelta > 0 {
		if proofbound.IsNonFinite(pl.ConstructionDelta) {
			return nil, fmt.Errorf(`%w: the loft fillet construction displacement is not finite`, decaderr.ErrUnsupported)
		}
		stationRound = proofbound.AbsSumUpper(stationRound, pl.ConstructionDelta)
		if proofbound.IsNonFinite(stationRound) {
			return nil, fmt.Errorf(`%w: the loft fillet station displacement is not finite`, decaderr.ErrUnsupported)
		}
	}
	if len(pl.ConstructionArcUpper) != 0 {
		if len(pairs) == 0 {
			return nil, fmt.Errorf(`%w: the loft fillet has no outer loop`, decaderr.ErrUnsupported)
		}
		counts := make(map[int]int, len(pl.ConstructionArcUpper))
		for _, segment := range pairs[0].Segment {
			if _, exists := pl.ConstructionArcUpper[segment]; exists {
				counts[segment]++
			}
		}
		for segment := range pl.ConstructionArcUpper {
			if counts[segment] == 0 {
				return nil, fmt.Errorf(`%w: the loft fillet arc-length key has no cell`, decaderr.ErrUnsupported)
			}
		}
		for cell, segment := range pairs[0].Segment {
			idealLength, exists := pl.ConstructionArcUpper[segment]
			if !exists {
				continue
			}
			if counts[segment] == 0 || idealLength <= 0 || proofbound.IsNonFinite(idealLength) {
				return nil, fmt.Errorf(`%w: the loft fillet arc-length bound is invalid`, decaderr.ErrUnsupported)
			}
			perCell := proofbound.UpRound(idealLength / float64(counts[segment]))
			pairs[0].ArcUpperV[cell] = max(pairs[0].ArcUpperV[cell], perCell)
			pairs[0].ArcUpperW[cell] = max(pairs[0].ArcUpperW[cell], perCell)
			energy := proofbound.UniformSpeedTangentEnergyUpper(perCell, 0)
			pairs[0].TangentEnergyV[cell] = max(pairs[0].TangentEnergyV[cell], energy)
			pairs[0].TangentEnergyW[cell] = max(pairs[0].TangentEnergyW[cell], energy)
		}
	}
	if len(pl.ConstructionLengthExtra) != 0 {
		if len(pairs) == 0 {
			return nil, fmt.Errorf(`%w: the loft fillet has no outer loop`, decaderr.ErrUnsupported)
		}
		seen := make(map[int]bool, len(pl.ConstructionLengthExtra))
		for cell, segment := range pairs[0].Segment {
			extra, exists := pl.ConstructionLengthExtra[segment]
			if !exists {
				continue
			}
			if extra < 0 || proofbound.IsNonFinite(extra) {
				return nil, fmt.Errorf(`%w: the loft fillet carrier-length bound is invalid`, decaderr.ErrUnsupported)
			}
			seen[segment] = true
			pairs[0].ArcUpperV[cell] = proofbound.AbsSumUpper(pairs[0].ArcUpperV[cell], extra)
			pairs[0].ArcUpperW[cell] = proofbound.AbsSumUpper(pairs[0].ArcUpperW[cell], extra)
			// The ideal carrier uses a slightly different normalized trim. Its
			// tangent energy is not the recorded carrier's energy, so use the
			// premise-free area leg until an exact ideal energy is derived.
			pairs[0].TangentEnergyV[cell] = math.Inf(1)
			pairs[0].TangentEnergyW[cell] = math.Inf(1)
		}
		for segment := range pl.ConstructionLengthExtra {
			if !seen[segment] {
				return nil, fmt.Errorf(`%w: the loft fillet carrier-length key has no cell`, decaderr.ErrUnsupported)
			}
		}
	}

	a, err := loftmesh.AssembleTriangulated(ctx, pairs, pl.Frame0, pl.Frame1, pl.Plane0, pl.Xform, stationRound)
	if err != nil {
		return nil, err
	}

	if err := loftmesh.LoftCrossingAuditStructured(budget, a.Verts, a.Tris, a.Walls, a.CapStartCount, a.VIdx, a.WIdx); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cap0Rat := loftmesh.CapPolygonAreaRat(a.Pts0, a.LoopIdx0)
	cap1Rat := loftmesh.CapPolygonAreaRat(a.Pts1, a.LoopIdx1)

	// A surface result publishes a sheet, never a solid: solid false is what
	// makes Volume() and Centroid() answer ErrNotSolid through their existing
	// guards, and kind BodySheet is what Kind() reports
	// (docs/surface-design.md §4.3), the same rule prismPayload's and
	// revolvePayload's own builds already follow (internal/prismrecord/build.go,
	// internal/revolveplan/build.go).
	kind := topo.BodySolid
	if pl.SurfaceResult {
		kind = topo.BodySheet
	}
	body := env.Alloc.NewBody()
	body.Solid = !pl.SurfaceResult
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Kind = kind
	body.Doc = env.Doc

	capStart, capEnd, walls, wallAreas, err := BuildTopology(ctx, env, body, a, cap0Rat, cap1Rat)
	if err != nil {
		return nil, err
	}
	// Section 0 lifts through frame0 and section 1 through frame1
	// (loftmesh.Assemble), so each cap denotes its own section plane.
	capStart.CapRecord = &topo.CapRecord{Frame: pl.Frame0, Xform: pl.Xform}
	capEnd.CapRecord = &topo.CapRecord{Frame: pl.Frame1, Xform: pl.Xform}
	for k, wall := range walls {
		li, cell := a.Cell[k][0], a.Cell[k][1]
		if li >= len(pl.BlendSegs) {
			continue // A through-bore adds an unblended hole wall.
		}
		if cell >= len(pairs[li].Segment) {
			return nil, fmt.Errorf(`%w: a loft fillet lost its segment correspondence`, decaderr.ErrUnsupported)
		}
		seg := pairs[li].Segment[cell]
		if _, blend := pl.BlendSegs[li][seg]; blend {
			wall.Origins = append(wall.Origins, provenance.Ref{Producer: ref, Role: fmt.Sprintf("fillet(%d,%d)", li, seg)})
		}
	}
	directCircle := DirectCircularHole(pl)
	var circularWall *topo.Face
	if pl.AnalyticBore || directCircle == WallCylinder {
		walls, err = buildAnalyticBoreTopology(ctx, env, body, pl, a, capStart, capEnd, walls)
		if err != nil {
			return nil, err
		}
	} else if directCircle != WallNone {
		walls, circularWall, err = buildVaryingCircularHoleTopology(ctx, env,
			body, pl, directCircle, a, capStart, capEnd, walls)
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// A surface result omits both caps from the shell (Table W,
	// docs/surface-design.md §4.1-§4.2): capStart/capEnd stay constructed above
	// so the area accumulation below can still read their area fields, but
	// neither joins the attached or published face set, which is what leaves
	// each section-plane rim edge with only its wall face free. topobuild.SheetLumps
	// splits the walls by connectivity — a holed loft's outer and hole wall
	// tubes touch nowhere once their shared cap is gone — and MUST run after
	// topobuild.AttachFaceLoops has registered every wall face on its own edges
	// (topobuild.ShellIsOpen's and topobuild.SheetLumps's doc comments); a solid loft keeps
	// the single-shell, single-lump body PR 1 already published, unchanged.
	if pl.SurfaceResult {
		if err := topobuild.AttachFaceLoops(ctx, walls); err != nil {
			return nil, err
		}
		body.Lumps = topobuild.SheetLumps(env.Alloc, walls)
	} else {
		faces := append([]*topo.Face{capStart, capEnd}, walls...)
		if err := topobuild.AttachFaceLoops(ctx, faces); err != nil {
			return nil, err
		}
		shell := env.Alloc.NewFaceShell()
		shell.Faces = faces
		lump := env.Alloc.NewLump()
		lump.Shells = []*topo.FaceShell{shell}
		body.Lumps = []*topo.Lump{lump}
	}

	mass := BuildMass(pl, a, pairs, sectionDelta, sectionMatchedDelta, wallAreas)
	body.Volume = massVolume(mass, a.Verts, a.Tris)
	centroid, err := massCentroid(mass, a.Verts, a.Tris)
	if err != nil {
		return nil, err
	}
	body.Centroid = centroid
	bounds, ok := massBounds(mass)
	if !ok {
		return nil, fmt.Errorf(`%w: the loft has no vertices to bound`, decaderr.ErrDegenerate)
	}
	body.Bounds = bounds
	body.Area = massArea(mass, cap0Rat, cap1Rat)
	if circularWall != nil {
		var wallArea proofbound.BoundedScalar
		if directCircle != WallCylinder {
			bottom, bottomOK := pl.Profile0.Holes[0].Segments[0].(sectionrecord.CircleSeg)
			top, topOK := pl.Profile1.Holes[0].Segments[0].(sectionrecord.CircleSeg)
			if !bottomOK || !topOK {
				return nil, fmt.Errorf(`%w: a circular loft wall lost its circle pair`, decaderr.ErrUnsupported)
			}
			var ok bool
			switch directCircle {
			case WallCone:
				wallArea, ok = coneWallArea(pl, bottom, top)
			case WallRuled:
				wallArea, ok = movingCircleWallArea(pl, bottom, top)
			case WallQuarterTurn:
				wallArea, ok = quarterTurnWallArea(pl, bottom, top)
			}
			if !ok {
				return nil, fmt.Errorf(`%w: the circular loft wall area has no finite bound`, decaderr.ErrUnsupported)
			}
		} else {
			// The Loft mass proof bounds the complete true surface. The two
			// circular-hole caps and every retained straight outer wall each
			// bound their own true areas, so outward subtraction isolates the
			// single curved hole wall without treating another face as exact.
			wallArea = proofbound.BoundedSub(
				proofbound.BoundedSub(measurementScalar(body.Area),
					proofbound.MeasuredScalar(capStart.Area, capStart.AreaBound)),
				proofbound.MeasuredScalar(capEnd.Area, capEnd.AreaBound))
			for _, face := range walls {
				if face == circularWall {
					continue
				}
				wallArea = proofbound.BoundedSub(wallArea,
					proofbound.MeasuredScalar(face.Area, face.AreaBound))
			}
		}
		if proofbound.IsNonFinite(wallArea.Value) || proofbound.IsNonFinite(wallArea.Bound) {
			return nil, fmt.Errorf(`%w: the circular loft wall area has no finite bound`, decaderr.ErrUnsupported)
		}
		circularWall.Area, circularWall.AreaBound = wallArea.Value, wallArea.Bound
	}
	if pl.SurfaceResult {
		// §4.3: a surface result's area is the solid's area minus the two
		// omitted caps'. Each cap's area and bound are read back from the
		// Face fields BuildTopology stamped, which already compose the
		// cap's own placement allowance (loftmesh.CapTriangleAreaAllow) — never
		// re-derived here — and the subtraction runs LAST, after the mass
		// accumulator's own area (walls plus both caps) has been read, so
		// nothing after this point still widens capStart's or capEnd's own
		// bound out from under a value already composed from them.
		// Collapsing this to the wall triangles' own sum instead (mass.area()
		// called with no cap terms) would hold the same VALUE — the two caps
		// cancel exactly, since they are the identical terms mass.area()
		// added in — but a SMALLER, uncomposed bound: the wall sum alone
		// carries none of the two caps' own rounding, which §4.3 requires
		// the sheet's bound to include.
		areaScalar := proofbound.BoundedSub(
			proofbound.BoundedSub(measurementScalar(body.Area), proofbound.MeasuredScalar(capStart.Area, capStart.AreaBound)),
			proofbound.MeasuredScalar(capEnd.Area, capEnd.AreaBound),
		)
		// Published Approximate EXPLICITLY, never derived from the composed
		// bound: the solid loft's own mass.area() always publishes
		// Approximate (loft_moments.go — a triangle's area is a square root,
		// never exactly representable), and a sheet built from the SAME wall
		// triangles must never claim more exactness than the body it came
		// from, whatever its own composed bound happens to round to.
		body.Area = measurement.Measurement{
			Value:     units.SquareMillimeters(areaScalar.Value),
			Exactness: measurement.Approximate,
			Bound:     units.SquareMillimeters(areaScalar.Bound),
		}
	}

	if err := topobuild.ValidateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}

	pl.Verts, pl.Tris, pl.Walls = a.Verts, a.Tris, a.Walls
	if pl.SourceIdentity == nil {
		pl.SourceIdentity = &SourceIdentity{Origin: ref}
	}
	pl.CapStartCount, pl.Cell, pl.Side = a.CapStartCount, a.Cell, a.Side
	pl.Segment = make([][]int, len(pairs))
	for i := range pairs {
		pl.Segment[i] = append([]int(nil), pairs[i].Segment...)
	}
	if pl.RetainTwistAreaCells {
		pl.TwistAreaCells = make([][]TwistAreaCellClaims, len(pairs))
		for i, pair := range pairs {
			pl.TwistAreaCells[i] = make([]TwistAreaCellClaims, len(pair.Segment))
			for j := range pair.Segment {
				pl.TwistAreaCells[i][j] = TwistAreaCellClaims{
					ArcLength: max(pair.ArcUpperV[j], pair.ArcUpperW[j]),
					Energy:    max(pair.TangentEnergyV[j], pair.TangentEnergyW[j]),
					Matched:   pair.MatchedDelta[j],
				}
			}
		}
	}
	pl.Proof = loftmesh.MeshProofOf(a, mass, sectionMatchedDelta)
	pl.Delta = a.Delta
	// sectionDelta is loftmesh.PairRecords' own accumulated MAX over cells
	// (Payload's own doc comment): zero for a LineSeg-only pairing,
	// positive for a same-kind circular one, to the sagitta its station
	// chording commits.
	pl.SectionDelta = sectionDelta
	pl.Chorded = loftmesh.IsChorded(pairs, sectionDelta, sectionMatchedDelta)
	body.Payload = pl
	return body, nil
}

// BuildMass folds the assembled triangle set into the mass accumulator
// and, on a chorded build, attaches the chorded proof terms. Build calls it
// once per build; it is its own function so the volume and area tests can read
// the same accumulator at a chord target of their choosing. A nil wallAreas
// reads every wall triangle afresh.
//
// docs/loft-design.md §5.2's matchedDelta row is composed here and nowhere
// else: proofbound.AbsSumUpper of the build's own MAX-over-cells chord-to-curve
// departure (loftmesh.PairRecords' sectionMatchedDelta) and the held vertex
// displacement a.Delta. The two halves are accumulated apart — a chord's
// departure from the curve it chords, and a station's departure from the
// point the record and the motion denote for it — and every chorded leg
// charges their SUM, since the chord the build DREW joins two displaced
// stations. The sagitta alone would leave the computed station's own
// displacement uncharged (§5.2's matchedDelta paragraph). Left at exactly 0 on
// a build with no chorded cell, which is what keeps a LineSeg-only pairing's
// published measurements free of every chorded term: its held triangle pair IS
// the boundary §5 gives it, and a.Delta already reaches its measurements
// through the accumulator's own delta-keyed legs.
//
// A build is chorded if any cell is not faceted, including a degree-1
// free-form cell with zero departure. Its bilinear patch still needs the
// exact correction and twist legs. The chorded terms remain zero for a
// LineSeg-only build, whose cells are all faceted.
func BuildMass(pl Payload, a loftmesh.Assembly, pairs []loftmesh.LoopPair, sectionDelta, sectionMatchedDelta float64, wallAreas []loftmesh.WallArea) *loftmesh.MassAccumulator {
	anchor := pl.Xform.Apply(pl.Plane0.Origin)
	chorded := loftmesh.IsChorded(pairs, sectionDelta, sectionMatchedDelta)
	matchedDelta := 0.0
	if chorded {
		matchedDelta = loftmesh.ChordCellDeltaUpper(sectionMatchedDelta, a.Delta)
	}
	mass := loftmesh.NewMassAccumulator(anchor, a.Delta, sectionDelta, matchedDelta)
	if wallAreas == nil {
		mass.AddMesh(a.Verts, a.Tris, a.Walls)
	} else {
		mass.AddMeshWithWallAreas(a.Verts, a.Tris, wallAreas)
	}
	if chorded {
		if wallAreas == nil {
			mass.Chorded = loftmesh.ComputeLoftChordedAllow(pairs, a.VIdx, a.WIdx, a.Verts, anchor, matchedDelta, a.Delta, a.Reversed)
		} else {
			mass.Chorded = loftmesh.ComputeLoftChordedAllowWithWallAreas(pairs, a.VIdx, a.WIdx, a.Verts, anchor, matchedDelta, a.Delta, a.Reversed, wallAreas)
		}
	}
	return mass
}

// The readings below adapt loftmesh's exact mass sums to the body's bounded
// measurements. A zero proven bound is the only Exact reading.

func massVolume(m *loftmesh.MassAccumulator, verts []r3.Vec, tris [][3]int) measurement.Measurement {
	value, bound := m.Volume(verts, tris)
	return measurement.Measurement{
		Value: units.CubicMillimeters(value), Exactness: measurement.ExactnessOf(bound),
		Bound: units.CubicMillimeters(bound),
	}
}

func massCentroid(m *loftmesh.MassAccumulator, verts []r3.Vec, tris [][3]int) (measurement.VecMeasurement, error) {
	value, bound, err := m.Centroid(verts, tris)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	return measurement.VecMeasurement{
		Value: value, Exactness: measurement.ExactnessOf(bound), Bound: units.Millimeters(bound),
	}, nil
}

func massBounds(m *loftmesh.MassAccumulator) (measurement.Box, bool) {
	lo, hi, bound, ok := m.Bounds()
	if !ok {
		return measurement.Box{}, false
	}
	return measurement.Box{Min: lo, Max: hi, Exactness: measurement.ExactnessOf(bound), Bound: units.Millimeters(bound)}, true
}

func massArea(m *loftmesh.MassAccumulator, capAreas ...*big.Rat) measurement.Measurement {
	value, bound := m.Area(capAreas...)
	return measurement.Measurement{
		Value: units.SquareMillimeters(value), Exactness: measurement.Approximate,
		Bound: units.SquareMillimeters(bound),
	}
}

// measurementScalar reads a measurement as a value and its proven bound.
func measurementScalar(m measurement.Measurement) proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(m.Value.Base(), m.Bound.Base())
}
