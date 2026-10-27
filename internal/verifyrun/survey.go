package verifyrun

import (
	"maps"

	"github.com/lestrrat-3d/decad/internal/analyticundercut"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/capblendsurvey"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/verifypublish"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file dispatches the wall, undercut, and minimum-radius questions of
// agent-docs/evaluator-design.md §10 and agent-docs/verification-design.md §6. It adapts
// private survey readings to public body and face handles. The analytic
// prism, revolve, and cup undercut decisions are in internal/analyticundercut;
// the cap-blend and draft decisions are in internal/capblendsurvey. The wall
// and concave-radius surveys require a proven solid. A proven-valid
// surface-extruded prism sheet can answer the undercut question about each
// wall's positive side (agent-docs/surface-design.md §2.3, §9.1). A payload this
// dispatch cannot decide leaves the requested question undecided.

// surveyReason retains why a survey was not decided. The zero reason is a
// numerical or geometric undecided result; surveyFacetedUnsupported records
// the known faceted-payload capability gap, and surveyPayloadStaged records
// every other explicit unsupported-payload dispatch — the deliberate
// cap-blend wall limit (DX9) and any payload class the wall, undercut, or
// concave-radius type switch does not name at all (today, a loft, and a draft
// body's wall and concave-radius surveys, draft DD8) — before
// runSurveys maps either into DiagUnsupportedSurveyPayload rather than a
// generic undecided result (proposal §16).
type surveyReason = verifypublish.SurveyReason

const (
	surveyUndecided          = verifypublish.SurveyUndecided
	surveyFacetedUnsupported = verifypublish.SurveyFacetedUnsupported
	surveyPayloadStaged      = verifypublish.SurveyPayloadStaged
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
	faces     []*topo.Face
	ok        bool
	undecided bool
	reason    surveyReason
}

// undercutFromNodes preserves face identity and the distinction between an
// empty proven all-clear and a nil undecided listing.
func undercutFromNodes(in verifypublish.UndercutSurvey[*topo.Face]) undercutOutcome {
	var faces []*topo.Face
	if in.Faces != nil {
		faces = make([]*topo.Face, len(in.Faces))
		copy(faces, in.Faces)
	}
	return undercutOutcome{faces: faces, ok: in.OK, undecided: in.Undecided, reason: in.Reason}
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
func facesByRole(b *topo.Body) map[string]*topo.Face {
	nodes := topobuild.FacesByRole(b)
	m := make(map[string]*topo.Face, len(nodes))
	maps.Copy(m, nodes)
	return m
}

// shellTol is the closed-form degeneracy tolerance for the cup wall offset.
const shellTol = offset2d.ShellTol

var cupWallOperations = cupwall.Operations{
	Offset: func(budget *proofbound.WorkBudget, profile momentinput.Profile, s, t float64) (momentinput.Profile, error) {
		return offset2d.OffsetProfile(budget, profile, s, t, shellTol)
	},
	Equal:   momentinput.ExactProfileEqual,
	Audit:   offset2d.AuditOffsetSectionBudget,
	Reverse: offset2d.ReverseLoopRecordBudget,
}

func cupWallInput(cp cupwall.View) cupwall.Input {
	return cupwall.Input{
		Outer: cp.Outer, Cavity: cp.Cavity,
		ZOpen: cp.ZOpen, ZOuter: cp.ZOuter, ZCav: cp.ZCav,
		Thickness: cp.Thickness, ThicknessDelta: cp.ThicknessDelta,
		Inward: cp.Sense == cupwall.Inward, Outward: cp.Sense == cupwall.Outward,
	}
}

// surveyResults is runSurveys' return record: the raw private outcome for
// each optional body survey, alongside whether cfg asked for it. It carries
// no legacy BodyReport reference, so runSurveys' geometry stays independent
// of the report shape internal/verifyrun/publish.go assembles from it.
//
// WallDiagnostics, UndercutDiagnostics, and RadiusDiagnostics are the exact
// subset of runSurveys' returned diagnostics that survey itself emitted (the
// same Diagnostic values, never recomputed): internal/verifyrun/publish.go routes each
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
// sheet (agent-docs/surface-design.md §2.3, §9.1): the wall and concave-radius
// blocks below run only on a topo.BodySolid, so a sheet leaves WallAsked and
// RadiusAsked false and their own prerequisite refusal to
// verifypublish.PublishWall/PublishRadius. It returns the raw private
// outcome for each survey and one diagnostic per
// non-Sound outcome: DiagWallTooThin / DiagUndercut (Violating) when a stated
// spec is proven to fail, and the per-survey DiagUndecided* (Suspect) when an
// asked question is undecided or a stated spec is straddled (verification
// §1.1/§6). A Sound survey emits nothing. Scalar readings are closed-form,
// each Exact only where its own arm proved a zero bound — a pinch reading or
// a sweep height with no axial displacement — and Approximate with the bound
// its own arithmetic derived otherwise; a cap-blend undercut survey can
// instead use a bounded normal range and leave an individual patch
// undecided. internal/verifyrun/publish.go's assembler maps the returned outcomes onto
// the private result vocabulary; this function never builds that vocabulary
// itself.
func runSurveys(budget *proofbound.WorkBudget, b *topo.Body, cfg verifyConfig) (surveyResults, []Diagnostic, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return surveyResults{}, nil, err
	}
	var diags []Diagnostic
	var results surveyResults

	if cfg.Wall != nil && b.Kind == topo.BodySolid {
		results.WallAsked = true
		out := wallOutcome{}
		var err error
		switch pl := b.Payload.(type) {
		case prismrecord.Payload:
			reading, readErr := wallsurvey.PrismWall(budget, pl.Profile,
				survey2d.PrismHeight{Z0: pl.Z0, Z1: pl.Z1, Z0Delta: pl.Z0Delta, Z1Delta: pl.Z1Delta},
				pl.SectionDelta, cfg.AllowRad)
			out, err = wallOutcome{reading: reading.Reading, bound: reading.Bound,
				ok: reading.OK, reason: reading.Reason}, readErr
		case revolveplan.Payload:
			reading, readErr := wallsurvey.RevolveWall(budget, wallsurvey.RevolveRecord{
				Profile: pl.Profile, Axis: pl.Ax, SectionDelta: pl.SectionDelta,
				Full: pl.Full, Phi0: pl.Phi0, Phi1: pl.Phi1, AngularDelta: pl.AngularDelta(),
			}, cfg.AllowRad)
			out, err = wallOutcome{reading: reading.Reading, bound: reading.Bound,
				ok: reading.OK, reason: reading.Reason}, readErr
		case cupwall.Payload:
			wall, wallErr := cupwall.Evaluate(budget, cupWallInput(pl.View()), cfg.AllowRad, cupWallOperations)
			out, err = wallOutcome{reading: wall.Reading, bound: wall.Bound, ok: wall.OK}, wallErr
		case capblend.Payload:
			// DX9 (agent-docs/modify-reach-design.md Table DX): a cap blend is not
			// one constant section at one height, so the existing 2D
			// spanning-disk proof does not decide it. A deliberate evaluator
			// limit, published as an explicit unsupported-payload dispatch
			// rather than a generic undecided result (proposal §16).
			out.reason = surveyPayloadStaged
		case facetedrecord.Payload:
			out, err = facetedWall(budget, pl)
		default:
			// Any payload class this switch does not name — a loft or a draft
			// body today (draft DD8) — has no implemented wall survey either.
			out.reason = surveyPayloadStaged
		}
		if err != nil {
			return surveyResults{}, nil, err
		}
		results.Wall = out
		wallDiags := verifypublish.WallDiagnostics[*topo.Body, vocab.JointCell](b, b.Payload,
			verifypublish.ScalarSurvey{Reading: out.reading, Bound: out.bound, OK: out.ok, Reason: out.reason},
			cfg.Wall.Tool, cfg.ToolMM)
		results.WallDiagnostics = wallDiags
		diags = append(diags, results.WallDiagnostics...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	if cfg.Pull != nil {
		results.UndercutAsked = true
		out := undercutOutcome{}
		switch pl := b.Payload.(type) {
		case prismrecord.Payload:
			out = undercutFromNodes(analyticundercut.Prism(b, pl, *cfg.Pull))
		case revolveplan.Payload:
			out = undercutFromNodes(analyticundercut.Revolve(b, pl, *cfg.Pull))
		case cupwall.Payload:
			out = undercutFromNodes(analyticundercut.Cup(b, pl.View(), *cfg.Pull))
		case capblend.Payload:
			out = undercutFromNodes(capblendsurvey.CapBlendUndercuts(b, pl, *cfg.Pull))
		case draftrecord.Payload:
			out = undercutFromNodes(capblendsurvey.DraftUndercuts(b, pl, *cfg.Pull))
		case breprecord.Payload:
			out = brepUndercuts(budget, b, pl, *cfg.Pull)
		case facetedrecord.Payload:
			var undercutErr error
			out, undercutErr = facetedUndercuts(budget, b, pl, *cfg.Pull)
			if undercutErr != nil {
				return surveyResults{}, nil, undercutErr
			}
		default:
			out.reason = surveyPayloadStaged
		}
		results.Undercut = out
		undercutDiags := verifypublish.UndercutDiagnostics[*topo.Body, *topo.Face, vocab.JointCell](b, b.Payload,
			verifypublish.UndercutSurvey[*topo.Face]{Faces: out.faces, OK: out.ok,
				Undecided: out.undecided, Reason: out.reason})
		results.UndercutDiagnostics = undercutDiags
		diags = append(diags, results.UndercutDiagnostics...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	if cfg.ConcaveRadius && b.Kind == topo.BodySolid {
		results.RadiusAsked = true
		out := radiusOutcome{}
		ok := false
		switch pl := b.Payload.(type) {
		case prismrecord.Payload:
			reading := radiussurvey.Prism(pl.Profile, pl.SectionDelta)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case revolveplan.Payload:
			reading := radiussurvey.Revolve(pl.Profile, pl.Ax, pl.SectionDelta)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case cupwall.Payload:
			view := pl.View()
			reading := radiussurvey.Cup(view.Outer, view.Cavity, view.OffsetDelta)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case capblend.Payload:
			reading := capblendsurvey.CapBlendMinRadius(b, pl)
			out, ok = radiusOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.OK}, reading.OK
		case breprecord.Payload:
			out, ok = brepMinRadius(b, pl)
		case facetedrecord.Payload:
			if o, found := orthogonalRadius(pl); found {
				out, ok = o, true
			} else {
				out.reason = surveyFacetedUnsupported
			}
		default:
			out.reason = surveyPayloadStaged
		}
		results.Radius = out
		radiusDiags := verifypublish.RadiusDiagnostics[*topo.Body, vocab.JointCell](b, b.Payload,
			verifypublish.ScalarSurvey{Reading: out.reading, Bound: out.bound, OK: out.ok, Reason: out.reason}, ok)
		results.RadiusDiagnostics = radiusDiags
		diags = append(diags, results.RadiusDiagnostics...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	return results, diags, nil
}
