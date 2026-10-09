package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolvesurvey"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is the analytic survey layer of
// docs/evaluator-design.md §10 and docs/verification-design.md §6: the wall,
// undercut and minimum-radius questions answered outright on the analytic
// prism, revolve and cup bodies. The wall and concave-radius surveys still
// need a proven solid; the undercut survey also answers on a proven-valid
// surface-extruded prism sheet, reading each wall's positive side in place
// of a solid's outward normal (docs/surface-design.md §2.3, §9.1) — its own
// cap loop is skipped, since a surface result publishes no cap face. Each
// survey reads the evaluator's own payload —
// the recorded 2D region a prism lifts or a revolve sweeps — because the
// spanning-ball and curvature readings are closed-form facts of that section:
// a prism's inscribed balls are its profile's inscribed disks with the height
// as the vertical fit, and a solid of revolution's are the disks of its
// meridian section (mirrored across the axis for a full turn, wedge-bounded
// by the caps for a partial one). A prism, cup, cap-blend and draft body's
// undercut survey reads each face's normal-component range enclosed over the
// rationals and decides it three-valued — clear, opposing, or undecided
// (survey_undercut.go, draft_survey.go) — rather than taking a float range at
// face value;
// revolveUndercuts is not yet converted to it, a deliberate scope line rather
// than an oversight. A body this file cannot decide — a payload no shipped
// feature builds — leaves every asked question undecided, which reads
// Suspect, never a silent pass.

// surveyReason retains why a survey was not decided. The zero reason is a
// numerical or geometric undecided result; surveyFacetedUnsupported records
// the known faceted-payload capability gap, and surveyPayloadStaged records
// every other explicit unsupported-payload dispatch — the deliberate
// cap-blend wall limit (DX9) and any payload class the wall, undercut, or
// concave-radius type switch does not name at all (today, a loft, and a draft
// body's wall and concave-radius surveys, draft DD8) — before
// runSurveys maps either into DiagUnsupportedSurveyPayload rather than a
// generic undecided result (proposal §16).
type surveyReason = reportvocab.SurveyReason

const (
	surveyUndecided          = reportvocab.SurveyUndecided
	surveyFacetedUnsupported = reportvocab.SurveyFacetedUnsupported
	surveyPayloadStaged      = reportvocab.SurveyPayloadStaged
)

// wallOutcome is one body's wall reading: ok=false is an undecided survey;
// reason distinguishes its cause. reading == nil (with ok) is the proven
// determination that no wall exists. bound is the PROVEN error bound beside
// reading (millimetres), zero only where the arm that produced reading is
// itself exactly representable — never asserted, always the arm's own
// computed figure.
type wallOutcome struct {
	reading *float64 // millimetres
	bound   float64  // millimetres; meaningful only when reading != nil
	ok      bool
	reason  surveyReason
}

// undercutOutcome is one body's undercut listing: ok=false is an unrecoverable
// undecided survey; reason distinguishes its cause. undecided records a
// straddling face that coexists with any faces already proven to oppose. faces
// is non-nil when every face is decided or any face is proven to oppose; an
// empty listing is the proven all-clear.
type undercutOutcome struct {
	faces     []*Face
	ok        bool
	undecided bool
	reason    surveyReason
}

// radiusOutcome is one body's tightest concave radius: ok=false is
// undecided and reason distinguishes its cause; reading == nil (with ok) is
// the proven all-convex answer. reading and bound (millimetres) are the
// midpoint and half-width of the interval the §9.2 survey2d.ExtremeAggregate proves over EVERY
// candidate the survey admitted, never one winning walk's own figures; the
// bound is zero, and the reading therefore Exact, only where every one of
// those candidates was itself exactly representable — a recorded CircleSeg's
// millimetre number, never a math.Hypot evaluation.
type radiusOutcome struct {
	reading *float64 // millimetres
	bound   float64  // millimetres; meaningful only when reading != nil
	ok      bool
	reason  surveyReason
}

// facesByRole indexes a body's faces by their own-step feature role.
func facesByRole(b *Body) map[string]*Face {
	step := b.origin.producer
	m := make(map[string]*Face)
	for _, f := range b.Faces() {
		for _, o := range f.origins {
			if o.producer == step {
				m[o.Role] = f
			}
		}
	}
	return m
}

// prismUndercuts surveys a prism's faces against the pull, through the exact
// three-valued reader survey_undercut.go shares with cupUndercuts and
// capBlendUndercuts's receiver-wall loop: planar sides and caps carry one
// normal each, cylindrical sides sweep their walk's exact angular range. A
// straddling face sets undecided without discarding a face already proven to
// oppose (docs/verification-design.md §6).
//
// A surface result (pp.surfaceResult) publishes no cap face at all
// (prism_build.go), so the cap loop below is skipped for one rather than
// probed and refused on a nil role lookup: on a SOLID a nil roleCapStart or
// roleCapEnd is a real build defect and the existing total refusal must
// still catch it, so the guard reads pp.surfaceResult, never f == nil. Every
// wall input this function reads — the placed frame's exact directions, the
// walk's plane-local tangent or circular range, the caller's pull — is
// unchanged on a sheet, because §2.3 gives a surface result's walls the
// orientation the solid would have had: the identical outward normal, from
// the identical code. The survey therefore quantifies over exactly the
// sheet's own published faces, which are its walls.
func prismUndercuts(b *Body, pp prismPayload, pull r3.Vec) undercutOutcome {
	if _, ok := pull.Normalize(); !ok {
		return undercutOutcome{}
	}
	m, okM := survey2d.NewPlacedFrameMap(pp.frame, pp.xform)
	if !okM {
		return undercutOutcome{}
	}
	roles := facesByRole(b)
	loops, err := boundarywalk.SurveyLoops(nil, boundarywalk.Profile(pp.profile))
	if err != nil {
		return undercutOutcome{}
	}
	faces := []*Face{}
	undecided := false
	for li, loop := range loops {
		for _, w := range loop {
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.Segs[0])]
			if f == nil {
				return undercutOutcome{}
			}
			verdict, ok := survey2d.WallNormalDecision(w, m, pull)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return undercutOutcome{}
			}
		}
	}
	if !pp.surfaceResult {
		for _, cap := range []struct {
			role string
			sign float64
		}{{role: roleCapStart, sign: -1}, {role: roleCapEnd, sign: 1}} {
			f := roles[cap.role]
			if f == nil {
				return undercutOutcome{}
			}
			verdict, ok := survey2d.CapNormalDecision(m, pull, cap.sign)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return undercutOutcome{}
			}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
}

// revolveUndercuts surveys a revolved body's faces against the pull: each
// wall's normal is n_ρ·radial(φ) + n_z·ŵ over the meridian range its walk
// sweeps and the azimuth range of the sweep — both exact.
func revolveUndercuts(b *Body, rp revolvePayload, pull r3.Vec) undercutOutcome {
	if rp.sectionDelta != 0 {
		// The survey reads the recorded meridian's own tangents as exact, and a
		// displaced meridian's tangents only sit within its displacement of the
		// denoted ones: a planar wall recorded off a rounded miter tilts by an
		// ulp and would list as an undercut. Undecided, which reads Suspect
		// (docs/surface-intersection-design.md §7.2).
		return undercutOutcome{}
	}
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	bas := rp.basis()
	pw := rp.xform.ApplyDir(bas.W).Dot(p)
	c0 := rp.xform.ApplyDir(bas.E0).Dot(p)
	c1 := rp.xform.ApplyDir(bas.E1).Dot(p)
	roles := facesByRole(b)
	loops, err := wallsurvey.RevolveLoops(nil, rp.profile, rp.ax.numeric())
	if err != nil {
		return undercutOutcome{}
	}
	faces := []*Face{}
	for _, decision := range revolvesurvey.UndercutRoles(
		loops, rp.ax, pw, c0, c1, rp.phi0, rp.phi1, rp.full, [2]string{roleCapStart, roleCapEnd},
	) {
		f := roles[decision.Role]
		if f == nil {
			return undercutOutcome{}
		}
		if decision.Opposes {
			faces = append(faces, f)
		}
	}
	return undercutOutcome{faces: faces, ok: true}
}

// cupWalks resolves one cup region loop into its coalesced walks — the same
// decomposition evalCup's wall build uses.
func cupWalks(loop loopRecord) ([]survey2d.SideWalk, error) {
	return cupWalksBudget(nil, loop)
}

func cupWalksBudget(budget *proofbound.WorkBudget, loop loopRecord) ([]survey2d.SideWalk, error) {
	loops, err := boundarywalk.SurveyLoopsBudget(budget, boundarywalk.Profile{Outer: loop})
	if err != nil {
		return nil, err
	}
	return loops[0], nil
}

var cupWallOperations = cupwall.Operations{
	Offset:  offsetProfile,
	Equal:   profileRecordsEqual,
	Audit:   auditOffsetSectionBudget,
	Reverse: offset2d.ReverseLoopRecordBudget,
}

func cupWallInput(cp cupView) cupwall.Input {
	return cupwall.Input{
		Outer: cp.outer, Cavity: cp.cavity,
		ZOpen: cp.zOpen, ZOuter: cp.zOuter, ZCav: cp.zCav,
		Thickness: cp.thickness, ThicknessDelta: cp.thicknessDelta,
		Inward: cp.sense == Inward, Outward: cp.sense == Outward,
	}
}

// cupUndercuts surveys a cup's faces against the pull (docs/modify-design.md
// D2): the outer walls over EVERY loop of region O (role side(i,j)), the cavity
// walls over every loop of the reversed region C (role shellSide(i,j)) — each
// read by the same exact three-valued reader the prism uses
// (survey_undercut.go), so a post wall is surveyed like any other — and the
// planar faces (the kept cap, the pocket floor, one rim per loop). The cavity
// being a pocket, the pocket floor (shellCap) and the rims face the open end
// and the kept cap (capStart) faces away from it, so their outward normals
// are ±N by which side of the outer floor the open end lies on.
func cupUndercuts(b *Body, cp cupView, pull r3.Vec) undercutOutcome {
	if _, ok := pull.Normalize(); !ok {
		return undercutOutcome{}
	}
	base := cp.basePrism()
	m, okM := survey2d.NewPlacedFrameMap(base.frame, base.xform)
	if !okM {
		return undercutOutcome{}
	}
	roles := facesByRole(b)

	faces := []*Face{}
	undecided := false
	survey := func(loop loopRecord, role string) bool {
		walks, err := cupWalks(loop)
		if err != nil {
			return false
		}
		for _, w := range walks {
			f := roles[fmt.Sprintf(role, w.Segs[0])]
			if f == nil {
				return false
			}
			verdict, ok := survey2d.WallNormalDecision(w, m, pull)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return false
			}
		}
		return true
	}
	oLoops := append([]loopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]loopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	for i, loop := range oLoops {
		if !survey(loop, fmt.Sprintf("side(%d,%%d)", i)) {
			return undercutOutcome{}
		}
	}
	for i, loop := range cLoops {
		crev, err := offset2d.ReverseLoopRecord(loop)
		if err != nil {
			return undercutOutcome{}
		}
		if !survey(crev, fmt.Sprintf("shellSide(%d,%%d)", i)) {
			return undercutOutcome{}
		}
	}

	// The planar faces. sOpen is +1 when the open end lies above the outer
	// floor: the kept cap then faces −N and the pocket floor and every rim
	// face +N. All rims share the open-end plane, so each has the same normal.
	sOpen := -1.0
	if cp.zOpen > cp.zOuter {
		sOpen = 1
	}
	caps := []struct {
		role string
		sign float64
	}{
		{role: roleCapStart, sign: -sOpen},
		{role: "shellCap", sign: sOpen},
	}
	for i := range oLoops {
		caps = append(caps, struct {
			role string
			sign float64
		}{role: fmt.Sprintf("rim(%d)", i), sign: sOpen})
	}
	for _, c := range caps {
		f := roles[c.role]
		if f == nil {
			return undercutOutcome{}
		}
		verdict, ok := survey2d.CapNormalDecision(m, pull, c.sign)
		if !listVerdict(&faces, &undecided, f, verdict, ok) {
			return undercutOutcome{}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
}

// surveyResults is runSurveys' return record: the raw private outcome for
// each optional body survey, alongside whether cfg asked for it. It carries
// no legacy BodyReport reference, so runSurveys' geometry stays independent
// of the report shape verify_publish.go assembles from it.
//
// WallDiagnostics, UndercutDiagnostics, and RadiusDiagnostics are the exact
// subset of runSurveys' returned diagnostics that survey itself emitted (the
// same Diagnostic values, never recomputed): verify_publish.go routes each
// onto its own result's Diagnostics (proposal §7, §10) so a body's wall,
// undercut, or concave-radius result carries its own findings without
// re-deriving them from the flat per-body list.
type surveyResults struct {
	WallAsked       bool
	Wall            wallOutcome
	WallDiagnostics []Diagnostic

	UndercutAsked       bool
	Undercut            undercutOutcome
	UndercutDiagnostics []Diagnostic

	RadiusAsked       bool
	Radius            radiusOutcome
	RadiusDiagnostics []Diagnostic
}

// runSurveys answers the asked opt-in questions on one proven-solid body, or
// the undercut question alone on a proven-valid surface-extruded prism
// sheet (docs/surface-design.md §2.3, §9.1): the wall and concave-radius
// blocks below run only on a BodySolid, so a sheet leaves WallAsked and
// RadiusAsked false and their own prerequisite refusal to
// reportvocab.PublishWall/PublishRadius. It returns the raw private
// outcome for each survey and one diagnostic per
// non-Sound outcome: DiagWallTooThin / DiagUndercut (Violating) when a stated
// spec is proven to fail, and the per-survey DiagUndecided* (Suspect) when an
// asked question is undecided or a stated spec is straddled (verification
// §1.1/§6). A Sound survey emits nothing. Scalar readings are closed-form,
// each Exact only where its own arm proved a zero bound — a pinch reading or
// a sweep height with no axial displacement — and Approximate with the bound
// its own arithmetic derived otherwise; a cap-blend undercut survey can
// instead use a bounded normal range and leave an individual patch
// undecided. verify_publish.go's assembler maps the returned outcomes onto
// the private result vocabulary; this function never builds that vocabulary
// itself.
func runSurveys(budget *proofbound.WorkBudget, b *Body, cfg verifyConfig) (surveyResults, []Diagnostic, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return surveyResults{}, nil, err
	}
	var diags []Diagnostic
	var results surveyResults

	if cfg.Wall != nil && b.Kind() == BodySolid {
		results.WallAsked = true
		out := wallOutcome{}
		var err error
		switch pl := b.payload.(type) {
		case prismPayload:
			reading, readErr := wallsurvey.PrismWall(budget, pl.profile,
				survey2d.PrismHeight{Z0: pl.z0, Z1: pl.z1, Z0Delta: pl.z0Delta, Z1Delta: pl.z1Delta},
				pl.sectionDelta, cfg.AllowRad)
			out, err = wallOutcome{reading: reading.Reading, bound: reading.Bound,
				ok: reading.OK, reason: reading.Reason}, readErr
		case revolvePayload:
			reading, readErr := wallsurvey.RevolveWall(budget, wallsurvey.RevolveRecord{
				Profile: pl.profile, Axis: pl.ax.numeric(), SectionDelta: pl.sectionDelta,
				Full: pl.full, Phi0: pl.phi0, Phi1: pl.phi1, AngularDelta: pl.angularDelta(),
			}, cfg.AllowRad)
			out, err = wallOutcome{reading: reading.Reading, bound: reading.Bound,
				ok: reading.OK, reason: reading.Reason}, readErr
		case cupPayload:
			wall, wallErr := cupwall.Evaluate(budget, cupWallInput(pl.view()), cfg.AllowRad, cupWallOperations)
			out, err = wallOutcome{reading: wall.Reading, bound: wall.Bound, ok: wall.OK}, wallErr
		case capBlendPayload:
			// DX9 (docs/modify-reach-design.md Table DX): a cap blend is not
			// one constant section at one height, so the existing 2D
			// spanning-disk proof does not decide it. A deliberate evaluator
			// limit, published as an explicit unsupported-payload dispatch
			// rather than a generic undecided result (proposal §16).
			out.reason = surveyPayloadStaged
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		default:
			// Any payload class this switch does not name — a loft or a draft
			// body today (draft DD8) — has no implemented wall survey either.
			out.reason = surveyPayloadStaged
		}
		if err != nil {
			return surveyResults{}, nil, err
		}
		results.Wall = out
		wallDiags := reportvocab.WallDiagnostics[*Body, JointCell](b, b.payload,
			reportvocab.ScalarSurvey{Reading: out.reading, Bound: out.bound, OK: out.ok, Reason: out.reason},
			cfg.Wall.Tool, cfg.ToolMM)
		results.WallDiagnostics = wallDiags
		diags = append(diags, wallDiags...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	if cfg.Pull != nil {
		results.UndercutAsked = true
		out := undercutOutcome{}
		switch pl := b.payload.(type) {
		case prismPayload:
			out = prismUndercuts(b, pl, *cfg.Pull)
		case revolvePayload:
			out = revolveUndercuts(b, pl, *cfg.Pull)
		case cupPayload:
			out = cupUndercuts(b, pl.view(), *cfg.Pull)
		case capBlendPayload:
			out = capBlendUndercuts(b, pl, *cfg.Pull)
		case draftPayload:
			out = draftUndercuts(b, pl, *cfg.Pull)
		case brepPayload:
			out = brepUndercuts(budget, b, pl, *cfg.Pull)
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		default:
			out.reason = surveyPayloadStaged
		}
		results.Undercut = out
		undercutDiags := reportvocab.UndercutDiagnostics[*Body, *Face, JointCell](b, b.payload,
			reportvocab.UndercutSurvey[*Face]{Faces: out.faces, OK: out.ok,
				Undecided: out.undecided, Reason: out.reason})
		results.UndercutDiagnostics = undercutDiags
		diags = append(diags, undercutDiags...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	if cfg.ConcaveRadius && b.Kind() == BodySolid {
		results.RadiusAsked = true
		out := radiusOutcome{}
		ok := false
		switch pl := b.payload.(type) {
		case prismPayload:
			reading := radiussurvey.Prism(pl.profile, pl.sectionDelta)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case revolvePayload:
			reading := radiussurvey.Revolve(pl.profile, pl.ax.numeric(), pl.sectionDelta)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case cupPayload:
			view := pl.view()
			reading := radiussurvey.Cup(view.outer, view.cavity, view.offsetDelta)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case capBlendPayload:
			out, ok = capBlendMinRadius(b, pl)
		case brepPayload:
			out, ok = brepMinRadius(b, pl)
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		default:
			out.reason = surveyPayloadStaged
		}
		results.Radius = out
		radiusDiags := reportvocab.RadiusDiagnostics[*Body, JointCell](b, b.payload,
			reportvocab.ScalarSurvey{Reading: out.reading, Bound: out.bound, OK: out.ok, Reason: out.reason}, ok)
		results.RadiusDiagnostics = radiusDiags
		diags = append(diags, radiusDiags...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	return results, diags, nil
}
