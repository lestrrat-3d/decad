package capblendsurvey

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/verifypublish"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/r3"
)

type prismPayload = prismrecord.Payload
type capPatch = capblend.Patch
type capPatchGeom = capband.Patch
type undercutOutcome = verifypublish.UndercutSurvey[*topo.Face]

// This file is the cap-blend payload's DX7/DX8 surveys
// (docs/modify-reach-design.md Table DX): per-patch normal ranges against a
// pull (undercut) and the minimum concave principal radius over the patch set
// (Table DX row DX8).
//
// Both rows turn on one fact about the band: a circular patch is RULED between
// two directrices, so the Cone it publishes is its surface only to within a
// proven bound (§8.3, internal/capblend/departure.go). DX7 widens its own reading by
// that bound. A provenly opposing point lists its patch; only a patch with no
// such point and a straddling range is undecided.
//
// The two rows read that fact through different halves of it. DX7's bound is a
// WORLD-space one and never reaches zero on a placed band: a mitered corner's
// angular skew is one half of it, and the placement's own independent rounding
// of every coordinate the build emits is the other, which leaves the tag even
// where the two windows coincide exactly. DX8 asks only whether the patch is a
// developable cone sector at all — a plane-local question the windows decide
// outright — and refuses for a band holding a mitered patch.
//
// DX7's reading owes a SECOND term, on every band and not only a mitered one:
// it reads each patch's normal through Face.NormalAt, whose answer is a
// direction the arm computed rather than the surface's own — a Cone's arm
// takes a float cosine and sine of the held half angle, which no held pair
// satisfies exactly — and it reads it at a POINT the survey itself computed,
// which is not the azimuth the reading is then treated as. A decision that
// keeps the sampled value and drops either gap answers for a direction the face
// never claimed, so PatchNormalRange charges the whole distance from its
// sampled reading to the patch's own exactly enclosed one (normal.go)
// as part of the allowance it returns. It is what keeps a pull the reading
// cannot separate from the patch's own tangent undecided rather than cleared,
// and no all-clear is proven outright unless BOTH terms are proven zero.
//
// DX9 (the wall survey) stays deliberately Suspect: a cap blend is not one
// constant section at one height, and the existing 2D spanning-disk proof
// does not decide it (runSurveys, internal/verifyrun/survey.go).

// CapBlendUndercuts surveys a cap-blend body's patches against the pull
// (DX7): each patch's published normal range is read from its OWN built Face —
// Face.NormalAt already carries the correct outward sign (the .reversed bit
// each patch sets at build time, internal/capblend/band.go) — sampled at enough
// azimuths to recover the patch's normal as A*cos(theta)+B*sin(theta)+C (a
// Cone's normal is that form in its own local azimuth; a Plane's is the
// degenerate A=B=0 case) and then read over the window [th0, th1] through a
// proven enclosure of that form's own extremes (normal.go). The
// ordinary (unchanged) side walls and caps are surveyed by the SAME exact
// three-valued reader analyticundercut.Prism runs (internal/survey2d/survey_undercut.go), since a
// cap-blend body's non-patch faces are built exactly like a prism's — and, as
// with a prism's own faces, an undecided receiver face sets the SAME
// undecided flag the patch loop below sets, so one body's two halves compose
// into one verdict.
//
// That range is the patch's own only where the patch's surface IS the one it
// publishes. A circular patch's is not, so its reading is widened by the
// departure the build already stamped on the face (internal/capblend/departure.go);
// every patch's reading is also widened by the whole distance its own readings
// can sit from the surface it publishes. Both widenings are composed inside
// PatchNormalRange, so the reading it returns is already the complete
// allowance — this survey reads it rather than composing it further. The
// existential listing and universal all-clear rules then decide it per point:
// see the loop below.
func CapBlendUndercuts(b *topo.Body, cbp capblend.Payload, pull r3.Vec) verifypublish.UndercutSurvey[*topo.Face] {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	roles := topobuild.FacesByRole(b)

	// The ordinary side walls and caps: the same exact three-valued read
	// analyticundercut.Prism runs, over the RECEIVER's own recorded profile — a
	// chamfered loop's unchanged (non-band) portion has the same wall role
	// and the same normal as an untouched one.
	pl := cbp.PrismLike(0, 0)
	m, okM := survey2d.NewPlacedFrameMap(pl.Frame, pl.Xform)
	if !okM {
		return undercutOutcome{}
	}
	loops, err := boundarywalk.SurveyLoops(nil, boundarywalk.Profile(cbp.Profile))
	if err != nil {
		return undercutOutcome{}
	}
	// Non-nil from the start: an EMPTY listing is this survey's proven
	// all-clear and a nil one is the undecided answer (BodyReport.Undercut.Faces,
	// internal/verifyrun/publish.go), so the two shapes must stay distinguishable — the
	// same distinction analyticundercut.Prism already keeps.
	faces := []*topo.Face{}
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
	for _, cap := range []struct {
		role string
		sign float64
	}{{role: provenance.RoleCapStart, sign: -1}, {role: provenance.RoleCapEnd, sign: 1}} {
		f := roles[cap.role]
		if f == nil {
			return undercutOutcome{}
		}
		verdict, ok := survey2d.CapNormalDecision(m, pull, cap.sign)
		if !listVerdict(&faces, &undecided, f, verdict, ok) {
			return undercutOutcome{}
		}
	}

	if !PatchUndercuts(roles, pl, cbp.Patches, p, &faces, &undecided) {
		return undercutOutcome{}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{Faces: faces, OK: true, Undecided: undecided}
}

// PatchUndercuts folds DX7's reading of every band patch into the running
// faces list and undecided flag, as listVerdict does for a receiver face. p is
// the unit pull. It reports false, a total refusal, when a patch's face is
// missing or its normal range cannot be enclosed. CapBlendUndercuts and
// DraftUndercuts (draft.go) share it, so a draft wall and a chamfer
// patch are read by one rule.
func PatchUndercuts(roles map[string]*topo.Face, pl prismPayload, patches []capPatch, p r3.Vec, faces *[]*topo.Face, undecided *bool) bool {
	// Read each patch's OWN built Face.NormalAt, walked in the payload's own
	// deterministic patch order (Table BX row BX3), so the faces this survey
	// reports — public output through Report.Bodies[i].Undercut.Faces — come
	// back in the same sequence on every call.
	for _, patch := range patches {
		f := roles[patch.Role]
		if f == nil {
			return false
		}
		mn, mx, reading, ok := PatchNormalRange(f, pl, patch.Geom, p)
		if !ok {
			return false
		}
		// reading already arrives complete: PatchNormalRange's circular arm
		// composes the patch's own departure from the surface it publishes
		// (internal/capblend/departure.go) into it directly, and its flat arm takes
		// that same departure already composed into Face.NormalAt's own
		// published bound (topology.go), since a flat patch has only the one
		// reading to widen. Composing f.NormalBound again here would charge a
		// flat patch's departure twice.
		allow := reading
		if allow <= 0 {
			// The patch's own surface IS the Cone (or Plane) it publishes AND
			// every reading it was assembled from is exact, so the range above
			// is exact and decides the patch outright.
			if survey2d.OpposesPull(mn, mx) {
				*faces = append(*faces, f)
			}
			continue
		}
		// Every point of the patch carries an azimuth inside this window, and
		// its own normal component sits within allow of the reading at that
		// azimuth: a CIRCULAR patch because the surface it publishes is its own
		// only to within its departure (internal/capblend/departure.go,
		// docs/modify-reach-design.md §8.3), and any patch at all because the
		// reading was assembled from bounded readings. A point proven to oppose
		// lists this patch; only an all-clear needs every point to clear. A
		// remaining straddle makes this patch undecided without discarding
		// other patches, receiver or patch, already proven to oppose —
		// survey2d.DecidePull's own three-valued rule (survey_undercut.go), read at this
		// patch's own allowance rather than the receiver faces' proven zero.
		switch survey2d.DecidePull(mn, mx, allow) {
		case survey2d.PullOpposes:
			*faces = append(*faces, f)
		case survey2d.PullUndecided:
			*undecided = true
		}
	}
	return true
}

// PatchNormalRange samples the published face and passes its exact normal
// model to capband, which encloses the circular patch's complete range.
func PatchNormalRange(f *topo.Face, pl prismPayload, g capPatchGeom, p r3.Vec) (float64, float64, float64, bool) {
	pLen, ok := capband.PullLengthUpper(p)
	if !ok {
		return 0, 0, 0, false
	}
	sampleAt := func(pt r3.Vec) (float64, float64, bool) {
		n, err := topobuild.NormalAt(f, pt)
		if err != nil {
			return 0, 0, false
		}
		return capband.PullComponent(n, p, pLen)
	}
	if !g.Circular {
		v, allow, ok := sampleAt(pl.Point(g.SideA.U, g.SideA.V, g.SideZ))
		if !ok {
			return 0, 0, 0, false
		}
		return v, v, allow, true
	}
	model, ok := capPatchNormalModel(f, pl, g, p)
	if !ok {
		return 0, 0, 0, false
	}
	r := g.CapRadius
	sample := func(theta float64) (float64, bool) {
		sin, cos := math.Sincos(theta)
		v, _, ok := sampleAt(pl.Point(g.CU+r*cos, g.CV+r*sin, g.CapZ))
		return v, ok
	}
	return capband.CircularNormalRange(g.Th0, g.Th1, g.WholeTurn, f.NormalBound, model, sample)
}

// CapBlendMinRadius is the tightest concave principal radius over a
// cap-blend body (Table DX, DX8).
//
// The reduction below is a claim about the patch the BUILD assembled, not
// merely about the tag it publishes: a Plane or Cone argument about "the
// patch's radius" is only a fact about the built surface where the built
// surface really is that Plane or Cone. The build already proves and stamps
// how far a patch's ruled surface can point away from the surface it
// publishes (`f.NormalBound`, `internal/capblend`'s `setPatchReadings`, derived
// in `internal/capblend/departure.go`), and this survey answers only where that stamp
// is an EXACT zero — a zero there means the built normal agrees with the
// tag's everywhere on the patch, so the two surfaces share every tangent plane
// and a boundary point, hence are the same surface, and the Plane/Cone
// argument below is a statement about the built patch rather than an
// assumption about it.
//
// `capblend.PatchWindowSkew` decides only the SKEW half of that departure — a
// question about the patch's own kind, whether the build rules it between two
// congruent windows at all. A MITERED circular patch fails even that: the
// build rules it between two differently-swept directrices
// (docs/modify-reach-design.md §8.3), and a straight-ruled surface between two
// skewed arcs is not developable at all — it carries curvature in both
// principal directions, tightening as the corner's own rulings converge, and
// neither the Cone argument below nor the receiver's own section says
// anything about it. But a coinciding window buys back only that half; the
// placement's own independent rounding of every emitted coordinate is the
// other half, and it is nonzero on every placed band, and a whole-turn Cone
// patch owes a further term even unplaced, since its own held half-angle only
// encloses a cosine and sine rather than fixing them exactly. So the skew
// check stays (it still rules out a mitered patch outright), but it is no
// longer sufficient on its own: a band is UNDECIDED here (`Suspect`, through
// runSurveys' own refusal diagnostic) unless every patch's own stamped
// departure is an exact zero, rather than answered with a proven absence the
// patch set does not support.
//
// For a band that passes, this slice's patches are Plane and Cone only — a
// chamfer produces no rolling-ball surface, so there is no Torus or Sphere
// case here at all — and NEITHER kind ever tightens the answer beyond what the
// receiver's own unchanged section already gives:
//
//   - a Plane patch has zero curvature in both principal directions (flat),
//     so it contributes no radius at all, exactly as a straight prism wall
//     never does (radiussurvey.Prism only ever reads circular walks);
//   - a regular Cone patch's ruling direction is a straight line (zero
//     curvature there too); its azimuthal principal radius is
//     R(z)/cos(halfAngle) >= R(z), so its tightest point (at the SIDE
//     boundary, R = the original wall's own radius) is never smaller than
//     that same wall's own unchanged radius — and SX7's band-reach gate
//     (buildCapBlend) guarantees a chamfered loop always keeps a strictly
//     positive unchanged run of that same wall, which radiussurvey.Prism reads
//     directly off the untouched RECEIVER profile;
//   - an apex-cone patch's radius shrinks to exactly zero only at its own
//     boundary VERTEX (the untouched original corner point) — a sharp
//     corner/edge feature, which this survey's own convention already
//     excludes everywhere else ("the survey reads faces' principal radii,
//     not edges", radiussurvey.Cup) — so reporting it here would
//     single out a reflex corner's un-rounded tip for a reading the SAME
//     corner, unchamfered, never received either.
//
// So for a band of those patches the correct answer is exactly "no new
// concave principal radius" and the whole survey reduces to radiussurvey.Prism on
// the receiver's own untouched profile.
func CapBlendMinRadius(b *topo.Body, cbp capblend.Payload) verifypublish.ScalarSurvey {
	roles := topobuild.FacesByRole(b)
	for _, patch := range cbp.Patches {
		f := roles[patch.Role]
		if f == nil {
			return verifypublish.ScalarSurvey{}
		}
		if capblend.PatchWindowSkew(patch.Geom) > 0 || f.NormalBound != 0 {
			return verifypublish.ScalarSurvey{}
		}
	}
	return radiussurvey.Prism(cbp.Profile, 0)
}
