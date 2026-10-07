package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolvesurvey"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
// by the caps for a partial one). A prism, cup and cap-blend body's undercut
// survey reads each face's normal-component range enclosed over the rationals
// and decides it three-valued — clear, opposing, or undecided
// (survey_undercut.go) — rather than taking a float range at face value;
// revolveUndercuts is not yet converted to it, a deliberate scope line rather
// than an oversight. A body this file cannot decide — a payload no shipped
// feature builds — leaves every asked question undecided, which reads
// Suspect, never a silent pass.

// surveyReason retains why a survey was not decided. The zero reason is a
// numerical or geometric undecided result; surveyFacetedUnsupported records
// the known faceted-payload capability gap, and surveyPayloadStaged records
// every other explicit unsupported-payload dispatch — the deliberate
// cap-blend wall limit (DX9) and any payload class the wall, undercut, or
// concave-radius type switch does not name at all (today, a loft) — before
// runSurveys maps either into DiagUnsupportedSurveyPayload rather than a
// generic undecided result (proposal §16).
type surveyReason int

const (
	surveyUndecided surveyReason = iota
	surveyFacetedUnsupported
	surveyPayloadStaged
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

// errFreeformSection is recordLoops' OWN name for requireAnalyticWalk's
// free-form refusal (docs/spline-design.md §8.1, Table R row R9), and the one
// error prismWall reads as an undecided wall reading rather than a failed
// survey. It wraps ErrUnsupported exactly as the refusal it renames does, so
// every other recordLoops consumer branches on it as before.
//
// It exists because "the section did not decompose" is not one cause: walkOf
// refuses a nil segment and a self-contradicting circle as ErrDegenerate,
// refuses a radius that is not a length on the unit conversion, and refuses a
// free-form span whose §5.2 work ceiling or §6.1 length bracket runs out as its
// OWN ErrUnsupported. Naming this one cause is what keeps a consumer's
// undecided reading from swallowing the rest.
var errFreeformSection = fmt.Errorf(`%w: the wall survey does not support a free-form boundary segment`, ErrUnsupported)

// recordLoops resolves the recorded profile into coalesced walk loops,
// exactly as the prism evaluator builds its side faces — the surveys must
// see the same face decomposition the topology carries.
func recordLoops(budget *proofbound.WorkBudget, profile ProfileRecord) ([][]survey2d.SideWalk, error) {
	// One free-form counter for the whole record: the surveys read a built body's
	// own section with no preflight counter in hand, so the ceiling starts here
	// and spans every loop below.
	work := freeform.NewFreeformWork()
	var out [][]survey2d.SideWalk
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			// requireAnalyticWalk refuses exactly one thing — a free-form walk
			// (extrude.go) — so its refusal returns under this file's own
			// sentinel, carrying the same message and the same ErrUnsupported.
			// Every other error above keeps its own identity.
			if err := requireAnalyticWalk(w, "the wall survey"); err != nil {
				return nil, errFreeformSection
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		}
		walks, err := coalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		out = append(out, walks)
	}
	return out, nil
}

// recordLoopsBudget names the operation-budget form used by cancellation
// probes and by callers that distinguish profile scanning from other surveys.
func recordLoopsBudget(budget *proofbound.WorkBudget, profile ProfileRecord) ([][]survey2d.SideWalk, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return nil, err
	}
	return recordLoops(budget, profile)
}

// revolveLoops resolves the loops into axis coordinates (the U fields carry
// z, the V fields ρ), mirroring buildRevolveLoop.
func revolveLoops(budget *proofbound.WorkBudget, rp revolvePayload) ([][]survey2d.SideWalk, error) {
	// One free-form counter for the whole record, as recordLoops opens.
	work := freeform.NewFreeformWork()
	var out [][]survey2d.SideWalk
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	for _, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := requireAnalyticWalk(w, "the survey boundary walk"); err != nil {
				return nil, err
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: rp.ax.walk(w), Segs: []int{i}}
		}
		walks, err := coalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		out = append(out, walks)
	}
	return out, nil
}

// walkElem keeps the root callers of survey2d.WalkElem on one conversion path.
// A free-form walk has no survey element and leaves its caller undecided.
func walkElem(w survey2d.SegmentWalk) (survey2d.SurveyElem, bool) {
	return survey2d.WalkElem(w)
}

// prismWall is the spanning-ball reading of a prism: the profile's spanning
// disks lift to balls when their diameter fits the height, the parallel caps
// span whenever a disk of half the height fits the section, and a profile
// corner within the allowance pinches to zero.
func prismWall(budget *proofbound.WorkBudget, pp prismPayload, alpha float64) (wallOutcome, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	if pp.sectionDelta != 0 {
		// A spanning ball fitted to the RECORDED section is not a proof about a
		// section that section is only within the payload's own displacement of
		// (docs/prism-boolean-design.md §7), and the reading carries no bound to
		// widen. Undecided, which reads Suspect — never a silent pass.
		return wallOutcome{}, nil
	}
	loops, err := recordLoops(budget, pp.profile)
	if err != nil {
		if errors.Is(err, errFreeformSection) {
			// A free-form boundary segment (docs/spline-design.md §8.1) is an
			// undecided wall reading, not a failed survey: Suspect, never a
			// silent pass and never a hard error out of Verify. This ONE
			// refusal is the whole of the reading's tolerance for a section it
			// cannot decompose — a degenerate segment, a radius that is not a
			// length, and a free-form span the evaluator's own work or range
			// ceiling refuses are failures, and reach the caller as themselves.
			//
			// A review read this nil return as swallowing a cancellation that
			// arrives after the poll above, so that the caller never learns of
			// it. The caller does learn: runSurveys is prismWall's only
			// production call site, and it polls survey2d.WallBudgetErr again on the
			// next unconditional line after the diagnostics, with no return
			// between; that budget's errFn is the Verify context's own Err,
			// which Verify then propagates. Checked in an isolated copy with a
			// context that returns nil for the first polls and then
			// context.Canceled: prismWall alone returns (undecided, nil), and
			// runSurveys on the same context returns context.Canceled.
			return wallOutcome{}, nil
		}
		return wallOutcome{}, err
	}
	height := survey2d.PrismHeight{Z0: pp.z0, Z1: pp.z1, Z0Delta: pp.z0Delta, Z1Delta: pp.z1Delta}
	reading, err := survey2d.PrismWallReading(budget, loops, height, alpha)
	return wallOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.Ok}, err
}

// revolveWall resolves the meridian walks and maps their bounded wall reading.
func revolveWall(budget *proofbound.WorkBudget, rp revolvePayload, alpha float64) (wallOutcome, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	loops, err := revolveLoops(budget, rp)
	if err != nil {
		return wallOutcome{}, err
	}
	reading, err := revolvesurvey.WallReading(budget, loops, rp.ax, rp.full, rp.phi0, rp.phi1, rp.angularDelta(), alpha)
	return wallOutcome{reading: reading.Reading, bound: reading.Bound, ok: reading.Ok}, err
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

// opposesPull decides the pointwise membership rule of verification §6 over
// a face's normal-component range [m, M] against the unit pull, read as a
// bare float with no allowance: a point opposes when its normal has a
// component against the pull — exactly perpendicular is not opposed — and a
// face whose normals are EXACTLY antiparallel everywhere separates under the
// pull rather than hooking it (the flat base a straight prism pulls off of).
// The carve-out is exact, never a tolerance band: a pull tilted by any real
// angle hooks under the base, however slightly, and §6 lists it. The clamp
// below absorbs only float overshoot past −1 in a unit dot; it never widens
// the exception.
//
// revolveUndercuts is this function's only remaining caller: prismUndercuts,
// cupUndercuts and capBlendUndercuts's receiver-wall loop instead read
// through survey2d.DecidePull's exact-rational sibling (survey_undercut.go), which
// this function's own zero-allowance behaviour matches exactly
// (TestDecidePullMatchesOpposesPullAtZeroAllowance).
func opposesPull(m, M float64) bool {
	if M < -1 {
		M = -1
	}
	return m < 0 && M > -1
}

// trigRange is the exact range of a·cosθ + b·sinθ over [lo, hi].
func trigRange(a, b, lo, hi float64) (float64, float64) {
	return clearance.TrigRange(a, b, lo, hi)
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
	m, okM := newPlacedFrameMap(pp)
	if !okM {
		return undercutOutcome{}
	}
	roles := facesByRole(b)
	loops, err := recordLoops(nil, pp.profile)
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
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	bas := rp.basis()
	pw := rp.xform.ApplyDir(bas.W).Dot(p)
	c0 := rp.xform.ApplyDir(bas.E0).Dot(p)
	c1 := rp.xform.ApplyDir(bas.E1).Dot(p)
	glo, ghi := sweepExtremes(c0, c1, rp.phi0, rp.phi1, rp.full)
	roles := facesByRole(b)
	loops, err := revolveLoops(nil, rp)
	if err != nil {
		return undercutOutcome{}
	}
	faces := []*Face{}
	for li, loop := range loops {
		for _, w := range loop {
			if rp.ax.classify(w.SegmentWalk) == wallAxis {
				continue
			}
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.Segs[0])]
			if f == nil {
				return undercutOutcome{}
			}
			mn, mx := math.Inf(1), math.Inf(-1)
			if w.IsCircular() {
				sigma := 1.0
				if w.Th1 < w.Th0 {
					sigma = -1
				}
				lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
				for _, g := range []float64{glo, ghi} {
					// n·p = σ(cosθ·pw + sinθ·g) over the meridian range.
					a, bb := trigRange(pw, g, lo, hi)
					mn = math.Min(mn, math.Min(sigma*a, sigma*bb))
					mx = math.Max(mx, math.Max(sigma*a, sigma*bb))
				}
			} else {
				l := math.Hypot(w.TanInU, w.TanInV)
				nz := w.TanInV / l
				nr := -w.TanInU / l
				for _, g := range []float64{glo, ghi} {
					v := nz*pw + nr*g
					mn = math.Min(mn, v)
					mx = math.Max(mx, v)
				}
			}
			if opposesPull(mn, mx) {
				faces = append(faces, f)
			}
		}
	}
	if !rp.full {
		sin0, cos0 := math.Sincos(rp.phi0)
		sin1, cos1 := math.Sincos(rp.phi1)
		for _, cap := range []struct {
			role string
			v    float64
		}{
			// A cap's outward normal is the sweep-velocity direction at its
			// own angle — against the sweep on the start cap, along it on
			// the end cap.
			{role: roleCapStart, v: -(c1*cos0 - c0*sin0)},
			{role: roleCapEnd, v: c1*cos1 - c0*sin1},
		} {
			f := roles[cap.role]
			if f == nil {
				return undercutOutcome{}
			}
			if opposesPull(cap.v, cap.v) {
				faces = append(faces, f)
			}
		}
	}
	return undercutOutcome{faces: faces, ok: true}
}

// radiusOutcomeOf is the concave-radius arms' wrapper over the same reduction:
// no candidate is the proven all-convex answer, an unresolvable interval is an
// undecided survey, and anything else is the aggregate's own reading.
func radiusOutcomeOf(ra survey2d.ExtremeAggregate) (radiusOutcome, bool) {
	if ra.Empty() && !ra.Unbounded {
		return radiusOutcome{ok: true}, true
	}
	mid, bound, ok := ra.Resolve()
	if !ok {
		return radiusOutcome{}, false
	}
	return radiusOutcome{reading: &mid, bound: bound, ok: true}, true
}

// prismMinRadius is the tightest concave radius over a prism's faces: only
// a side swept from an arc walked against the section's orientation — a
// hole wall, a notch — curves away from the material, and its radius is the
// walk's own. Every such walk is a candidate the §9.2 survey2d.ExtremeAggregate reduces; the
// reading is that aggregate's interval, never one walk's own figures.
func prismMinRadius(pp prismPayload) (radiusOutcome, bool) {
	if pp.sectionDelta != 0 {
		// The reading is a radius READ OFF the recorded section, and a payload
		// carrying a section displacement (docs/prism-boolean-design.md §7)
		// denotes a section its own record is only within that displacement of.
		// The survey answers a measurement with no bound of its own, so it leaves
		// the question undecided rather than publishing a radius for the wrong
		// section.
		return radiusOutcome{}, false
	}
	loops, err := recordLoops(nil, pp.profile)
	if err != nil {
		return radiusOutcome{}, false
	}
	agg := survey2d.MinAggregate()
	for _, loop := range loops {
		for _, w := range loop {
			if w.IsCircular() && w.Th1 < w.Th0 {
				// Each concave arc enters under its OWN proven radius bound
				// (extrude.go's arcWalkRadiusBound), and the aggregate — not
				// this loop — decides what the reading says.
				agg.Take(w.Radius, w.RadiusBound)
			}
		}
	}
	return radiusOutcomeOf(agg)
}

// revolveMinRadius resolves the meridian walks for their bounded curvature reading.
func revolveMinRadius(rp revolvePayload) (radiusOutcome, bool) {
	loops, err := revolveLoops(nil, rp)
	if err != nil {
		return radiusOutcome{}, false
	}
	return radiusOutcomeOf(revolvesurvey.RadiusAggregate(loops, rp.ax))
}

// cupWalks resolves one cup region loop into its coalesced walks — the same
// decomposition evalCup's wall build uses.
func cupWalks(loop LoopRecord) ([]survey2d.SideWalk, error) {
	return cupWalksBudget(nil, loop)
}

func cupWalksBudget(budget *proofbound.WorkBudget, loop LoopRecord) ([]survey2d.SideWalk, error) {
	loops, err := recordLoopsBudget(budget, ProfileRecord{Outer: loop})
	if err != nil {
		return nil, err
	}
	return loops[0], nil
}

// cupWall returns the shell-wall theorem of
// docs/payload-verification-design.md §4: an accepted cup reads its shell
// thickness unless one of its material junctions is within the caller's draft
// allowance, in which case the closure-under-limits rule makes the reading
// exactly zero. The thickness reading carries the payload's own
// millimetre-conversion displacement as its bound and is Exact only when that
// displacement is zero; the pinch reading is always Exact zero. The theorem
// consumes the payload's morphology, not caller input: it rebuilds and
// audits the offset relation before trusting it.
func cupWall(budget *proofbound.WorkBudget, cp cupPayload, alpha float64) (wallOutcome, error) {
	finite := func(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
	isCancellation := func(err error) bool {
		return errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded)
	}
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	t := cp.thickness
	if !finite(t) || t <= 0 || (cp.sense != Inward && cp.sense != Outward) {
		return wallOutcome{}, nil
	}
	dOuter := cp.zOpen - cp.zOuter
	dCavity := cp.zOpen - cp.zCav
	if !finite(dOuter) || !finite(dCavity) || dOuter == 0 || dCavity == 0 ||
		math.Signbit(dOuter) != math.Signbit(dCavity) || math.Abs(dCavity) >= math.Abs(dOuter) {
		return wallOutcome{}, nil
	}
	openDir := 1.0
	if dOuter < 0 {
		openDir = -1
	}
	if cp.sense == Inward && cp.zCav != cp.zOuter+openDir*t {
		return wallOutcome{}, nil
	}
	if cp.sense == Outward && cp.zOuter != cp.zCav-openDir*t {
		return wallOutcome{}, nil
	}

	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return wallOutcome{}, nil
	}
	if oi, err := cp.outer.integralsBudget(budget); err != nil {
		if isCancellation(err) {
			return wallOutcome{}, err
		}
		return wallOutcome{}, nil
	} else if oi.area <= 0 || !finite(oi.area) {
		return wallOutcome{}, nil
	}
	if ci, err := cp.cavity.integralsBudget(budget); err != nil {
		if isCancellation(err) {
			return wallOutcome{}, err
		}
		return wallOutcome{}, nil
	} else if ci.area <= 0 || !finite(ci.area) {
		return wallOutcome{}, nil
	}

	// Inward cups store C = O ⊖ t. Outward cups store O = C ⊕ t. Structural
	// equality is the exact claim: a residual or tolerance match could only
	// guess that the regions correspond.
	offsetMatches := func(orig, want ProfileRecord, sense float64) (bool, error) {
		got, err := offsetProfile(budget, orig, sense, t)
		if err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		same, err := profileRecordsEqual(budget, got, want)
		if err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		if !same {
			return false, nil
		}
		if err := auditOffsetSectionBudget(budget, orig, got); err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		return true, nil
	}
	matches, err := offsetMatches(cp.outer, cp.cavity, 1)
	if err != nil {
		return wallOutcome{}, err
	}
	if cp.sense == Outward {
		matches, err = offsetMatches(cp.cavity, cp.outer, -1)
		if err != nil {
			return wallOutcome{}, err
		}
	}
	if !matches {
		return wallOutcome{}, nil
	}

	hasPinch := func(loops [][]survey2d.SideWalk) (bool, bool, error) {
		for _, loop := range loops {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return false, false, err
			}
			if len(loop) == 0 {
				return false, false, nil
			}
			if len(loop) == 1 && loop[0].Closed {
				continue
			}
			for i, w := range loop {
				if err := survey2d.WallBudgetStep(budget); err != nil {
					return false, false, err
				}
				prev := loop[(i+len(loop)-1)%len(loop)]
				if survey2d.JunctionPinch(prev.TanOutU, prev.TanOutV, w.TanInU, w.TanInV, alpha) {
					return true, true, nil
				}
			}
		}
		return false, true, nil
	}

	outerWalks, err := recordLoopsBudget(budget, cp.outer)
	if err != nil {
		return wallOutcome{}, err
	}
	pinch, ok, err := hasPinch(outerWalks)
	if err != nil {
		return wallOutcome{}, err
	}
	if !ok {
		return wallOutcome{}, nil
	} else if pinch {
		zero := 0.0
		return wallOutcome{reading: &zero, ok: true}, nil
	}

	// The cavity boundary is a void skin, so reverse every recorded loop to
	// restore the same material-left walk convention junctionPinch expects.
	var cavityWalks [][]survey2d.SideWalk
	for _, loop := range cLoops {
		reversed, err := reverseLoopRecordBudget(budget, loop)
		if err != nil {
			return wallOutcome{}, err
		}
		walks, err := cupWalksBudget(budget, reversed)
		if err != nil {
			return wallOutcome{}, err
		}
		cavityWalks = append(cavityWalks, walks)
	}
	pinch, ok, err = hasPinch(cavityWalks)
	if err != nil {
		return wallOutcome{}, err
	}
	if !ok {
		return wallOutcome{}, nil
	} else if pinch {
		zero := 0.0
		return wallOutcome{reading: &zero, ok: true}, nil
	}

	return wallOutcome{reading: &t, bound: cp.thicknessDelta, ok: true}, nil
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
func cupUndercuts(b *Body, cp cupPayload, pull r3.Vec) undercutOutcome {
	if _, ok := pull.Normalize(); !ok {
		return undercutOutcome{}
	}
	base := cp.basePrism()
	m, okM := newPlacedFrameMap(base)
	if !okM {
		return undercutOutcome{}
	}
	roles := facesByRole(b)

	faces := []*Face{}
	undecided := false
	survey := func(loop LoopRecord, role string) bool {
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
	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	for i, loop := range oLoops {
		if !survey(loop, fmt.Sprintf("side(%d,%%d)", i)) {
			return undercutOutcome{}
		}
	}
	for i, loop := range cLoops {
		crev, err := reverseLoopRecord(loop)
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

// cupMinRadius is the tightest concave radius over a cup's faces (D3): the same
// walk the prism runs, over every loop of the outer region O and every loop of
// the cavity region C read reversed. A wall that curves away from the material
// — the cavity's reversed outer (a pocket wall), every hole of O (a tunnel) —
// contributes its concave radius; a wall that curves toward it — the outer
// region's own convex rounds, a post's outer cylinder (the reversed cavity
// hole) — is not a concave feature and rightly does not appear. Each loop is
// placed with the same walk sense evalCup builds it in, so prismMinRadius reads
// concavity off the walk direction, exactly as on a prism. The sharp concave
// edge where a wall meets the floor carries no radius — the survey reads faces'
// principal radii, not edges. The offset region's arcs are recorded within the
// cup's offsetDelta of the arcs it denotes: each shares its centre with the
// denoted arc and its start sits within that displacement of the denoted
// circle, so every recorded radius is within offsetDelta of its denoted one
// and the reading's bound takes it.
func cupMinRadius(cp cupPayload) (radiusOutcome, bool) {
	profile := ProfileRecord{Outer: cp.outer.Outer}
	profile.Holes = append(profile.Holes, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	for _, loop := range cLoops {
		crev, err := reverseLoopRecord(loop)
		if err != nil {
			return radiusOutcome{}, false
		}
		profile.Holes = append(profile.Holes, crev)
	}
	out, ok := prismMinRadius(prismPayload{profile: profile})
	if ok && out.ok && out.reading != nil && cp.offsetDelta > 0 {
		out.bound = proofbound.AbsSumUpper(out.bound, cp.offsetDelta)
	}
	return out, ok
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
// publishWallResult/publishConcaveRadiusResult. It returns the raw private
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

	if cfg.wall != nil && b.Kind() == BodySolid {
		results.WallAsked = true
		out := wallOutcome{}
		var err error
		switch pl := b.payload.(type) {
		case prismPayload:
			out, err = prismWall(budget, pl, cfg.allowRad)
		case revolvePayload:
			out, err = revolveWall(budget, pl, cfg.allowRad)
		case cupPayload:
			out, err = cupWall(budget, pl, cfg.allowRad)
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
			// Any payload class this switch does not name — a loft today —
			// has no implemented wall survey either.
			out.reason = surveyPayloadStaged
		}
		if err != nil {
			return surveyResults{}, nil, err
		}
		results.Wall = out
		var wallDiags []Diagnostic
		switch {
		case !out.ok:
			wallDiags = append(wallDiags, surveyRefusalDiagnostic(
				b,
				SurveyWall,
				out.reason,
				DiagUndecidedWall,
				"the wall survey could neither answer nor prove no wall exists",
				"facetedPayload wall survey support is not implemented; use an analytic body or wait for faceted wall support",
				"wall",
			))
		case out.reading != nil:
			m := lengthMeasurement(*out.reading, out.bound)
			tool := cfg.wall.tool
			switch intervalVerdict(*out.reading, out.bound, cfg.toolMM) {
			case -1:
				obs := m
				wallDiags = append(wallDiags, Diagnostic{
					Code:     DiagWallTooThin,
					Status:   Violating,
					Body:     b,
					Survey:   SurveyWall,
					Reading:  ReadingWall,
					Observed: &obs,
					Required: &tool,
					Message:  "the minimum wall thickness is proven below the tool",
				})
			case 0:
				obs := m
				wallDiags = append(wallDiags, Diagnostic{
					Code:     DiagUndecidedWall,
					Status:   Suspect,
					Body:     b,
					Survey:   SurveyWall,
					Reading:  ReadingWall,
					Observed: &obs,
					Required: &tool,
					Message:  "the minimum wall thickness interval straddles the tool",
				})
			}
		}
		results.WallDiagnostics = wallDiags
		diags = append(diags, wallDiags...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	if cfg.pull != nil {
		results.UndercutAsked = true
		out := undercutOutcome{}
		switch pl := b.payload.(type) {
		case prismPayload:
			out = prismUndercuts(b, pl, *cfg.pull)
		case revolvePayload:
			out = revolveUndercuts(b, pl, *cfg.pull)
		case cupPayload:
			out = cupUndercuts(b, pl, *cfg.pull)
		case capBlendPayload:
			out = capBlendUndercuts(b, pl, *cfg.pull)
		case brepPayload:
			out = brepUndercuts(b, pl, *cfg.pull)
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		default:
			out.reason = surveyPayloadStaged
		}
		results.Undercut = out
		var undercutDiags []Diagnostic
		if out.ok {
			if len(out.faces) > 0 {
				// An undercut is a predicate, not a scalar; the outcome's own
				// face list already names them, so the pair emits one
				// DiagUndercut naming the body.
				undercutDiags = append(undercutDiags, Diagnostic{
					Code:    DiagUndercut,
					Status:  Violating,
					Body:    b,
					Survey:  SurveyUndercut,
					Reading: ReadingNone,
					Message: "a face is a proven undercut against the pull",
				})
			}
		}
		if !out.ok || out.undecided {
			undercutDiags = append(undercutDiags, surveyRefusalDiagnostic(
				b,
				SurveyUndercut,
				out.reason,
				DiagUndecidedUndercut,
				"the pull survey could neither prove nor exclude an undercut",
				"facetedPayload pull survey support is not implemented; use an analytic body or wait for faceted undercut support",
				"pull",
			))
		}
		results.UndercutDiagnostics = undercutDiags
		diags = append(diags, undercutDiags...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	if cfg.concaveRadius && b.Kind() == BodySolid {
		results.RadiusAsked = true
		out := radiusOutcome{}
		ok := false
		switch pl := b.payload.(type) {
		case prismPayload:
			out, ok = prismMinRadius(pl)
		case revolvePayload:
			out, ok = revolveMinRadius(pl)
		case cupPayload:
			out, ok = cupMinRadius(pl)
		case capBlendPayload:
			out, ok = capBlendMinRadius(b, pl)
		case brepPayload:
			out, ok = brepMinRadius(pl)
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		default:
			out.reason = surveyPayloadStaged
		}
		results.Radius = out
		var radiusDiags []Diagnostic
		if !ok || !out.ok {
			radiusDiags = append(radiusDiags, surveyRefusalDiagnostic(
				b,
				SurveyConcaveRadius,
				out.reason,
				DiagUndecidedMinRadius,
				"the concave-radius survey could neither measure nor exclude a concave feature",
				"facetedPayload concave-radius survey support is not implemented; use an analytic body or wait for faceted radius support",
				"concave-radius",
			))
		}
		results.RadiusDiagnostics = radiusDiags
		diags = append(diags, radiusDiags...)
		if err := survey2d.WallBudgetErr(budget); err != nil {
			return surveyResults{}, nil, err
		}
	}

	return results, diags, nil
}

// surveyRefusalDiagnostic builds the one diagnostic an asked survey emits
// when it cannot answer at all (verification §1.1): undecidedCode/Message for
// a numerically or geometrically undecided proof, DiagUnsupportedSurveyPayload
// for a known payload capability gap — facetedMessage for a facetedPayload
// operand, or a message this function derives from the body's own payload
// type and surveyNoun (e.g. "wall", "pull", "concave-radius") for any other
// explicit unsupported-payload dispatch (proposal §16).
func surveyRefusalDiagnostic(
	body *Body,
	survey SurveyKind,
	reason surveyReason,
	undecidedCode DiagnosticCode,
	undecidedMessage string,
	facetedMessage string,
	surveyNoun string,
) Diagnostic {
	code, message := undecidedCode, undecidedMessage
	switch reason {
	case surveyFacetedUnsupported:
		code, message = DiagUnsupportedSurveyPayload, facetedMessage
	case surveyPayloadStaged:
		class := payloadClassName(body.payload)
		code = DiagUnsupportedSurveyPayload
		message = fmt.Sprintf(
			"%s %s survey support is not implemented; use an analytic body or wait for wider %s survey support",
			class, surveyNoun, class)
	}
	return Diagnostic{
		Code:    code,
		Status:  Suspect,
		Body:    body,
		Survey:  survey,
		Reading: ReadingNone,
		Message: message,
	}
}

// payloadClassName names payload's Go type without its package qualifier —
// the bare shape ("facetedPayload", "loftPayload") an unsupported-survey
// diagnostic message already uses.
func payloadClassName(payload any) string {
	name := fmt.Sprintf("%T", payload)
	if i := strings.LastIndexByte(name, '.'); i >= 0 {
		return name[i+1:]
	}
	return name
}

// lengthMeasurement wraps a survey reading with the PROVEN bound its own arm
// computed: Exact only where that arm's own arithmetic proved bound zero,
// Approximate otherwise — never asserted (docs/verification-design.md §6).
func lengthMeasurement(mm, bound float64) Measurement {
	return Measurement{Value: units.Millimeters(mm), Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}
}

// intervalVerdict decides a stated spec on the proven interval
// [value − bound, value + bound] against the tool (verification §6): −1 is
// proven thin (every admissible thickness under the tool), +1 is met
// (exactly tool-thick is not thinner), 0 is a straddle — undecided.
func intervalVerdict(value, bound, tool float64) int {
	if value+bound < tool {
		return -1
	}
	if value-bound >= tool {
		return 1
	}
	return 0
}
