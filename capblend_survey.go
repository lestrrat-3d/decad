package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is the cap-blend payload's DX7/DX8 surveys
// (docs/modify-reach-design.md Table DX): per-patch normal ranges against a
// pull (undercut) and the minimum concave principal radius over the patch set
// (Table DX row DX8).
//
// Both rows turn on one fact about the band: a circular patch at a mitered
// corner is RULED between two differently-swept directrices, so the Cone it
// publishes is its surface only to within a proven bound (§8.3). DX7 widens
// its own reading by that bound and answers only where the widened interval
// still falls on one side of the rule; DX8 does not answer for such a band at
// all.
//
// DX9 (the wall survey) stays deliberately Suspect: a cap blend is not one
// constant section at one height, and the existing 2D spanning-disk proof
// does not decide it (runSurveys, survey.go).

// capBlendUndercuts surveys a cap-blend body's patches against the pull
// (DX7): each patch's exact normal range is read from its OWN built Face —
// Face.NormalAt already carries the correct outward sign (the .reversed bit
// each patch sets at build time, capblend_geom.go) — sampled at enough
// azimuths to recover the patch's normal as A*cos(theta)+B*sin(theta)+C
// exactly (a Cone's normal is that form in its own local azimuth; a Plane's
// is the degenerate A=B=0 case) and then read over the window [th0, th1]
// with the same closed-form trigRange the revolve undercut survey already
// uses. The ordinary (unchanged) side walls and caps are surveyed by the
// SAME per-role wallNormalRange logic prismUndercuts already runs, since a
// cap-blend body's non-patch faces are built exactly like a prism's.
//
// That range is the patch's own only where the patch's surface IS the one it
// publishes. A mitered circular patch's is not, so its reading is widened by
// its own proven departure (capPatchNormalAllow) and decides the patch only
// where the widened interval stays on one side of the rule: see the loop
// below.
func capBlendUndercuts(b *Body, cbp capBlendPayload, pull r3.Vec) undercutOutcome {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	roles := facesByRole(b)

	// The ordinary side walls and caps: the same per-role normal-range read
	// prismUndercuts runs, over the RECEIVER's own recorded profile — a
	// chamfered loop's unchanged (non-band) portion has the same wall role
	// and the same normal as an untouched one.
	pl := cbp.prismLike(0, 0)
	du := pl.dir(1, 0, 0).Dot(p)
	dv := pl.dir(0, 1, 0).Dot(p)
	dn := pl.dir(0, 0, 1).Dot(p)
	loops, err := recordLoops(nil, cbp.profile)
	if err != nil {
		return undercutOutcome{}
	}
	// Non-nil from the start: an EMPTY listing is this survey's proven
	// all-clear and a nil one is the undecided answer (BodyReport.Undercuts,
	// verify.go), so the two shapes must stay distinguishable — the same
	// distinction prismUndercuts already keeps.
	faces := []*Face{}
	for li, loop := range loops {
		for _, w := range loop {
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.segs[0])]
			if f == nil {
				return undercutOutcome{}
			}
			mn, mx := wallNormalRange(w, du, dv)
			if opposesPull(mn, mx) {
				faces = append(faces, f)
			}
		}
	}
	for _, cap := range []struct {
		role string
		v    float64
	}{{role: roleCapStart, v: -dn}, {role: roleCapEnd, v: dn}} {
		f := roles[cap.role]
		if f == nil {
			return undercutOutcome{}
		}
		if opposesPull(cap.v, cap.v) {
			faces = append(faces, f)
		}
	}

	// The new patches: read each one's OWN built Face.NormalAt, walked in the
	// payload's own deterministic patch order (Table BX row BX3), so the faces
	// this survey reports — public output through Report.Bodies[i].Undercuts —
	// come back in the same sequence on every call.
	for _, patch := range cbp.patches {
		f := roles[patch.role]
		if f == nil {
			return undercutOutcome{}
		}
		mn, mx, ok := capPatchNormalRange(f, pl, patch.geom, p)
		if !ok {
			return undercutOutcome{}
		}
		allow := capPatchNormalAllow(patch.geom)
		if allow <= 0 {
			// The patch's own surface IS the Cone (or Plane) it publishes, so
			// the range above is exact and decides the patch outright.
			if opposesPull(mn, mx) {
				faces = append(faces, f)
			}
			continue
		}
		// A MITERED circular patch is ruled between two differently-swept
		// directrices, so the surface it publishes is its own only to within
		// allow (capblend_geom.go, docs/modify-reach-design.md §8.3): every
		// point of the patch carries an azimuth inside this window, and its own
		// normal component sits within allow of the reading at that azimuth. So
		// the patch is decided only where the WHOLE widened interval falls on
		// one side of the rule, and undecided otherwise — never passed on the
		// unwidened reading, which would report a Sound the geometry has not
		// earned.
		switch {
		case mn > allow:
			// Every point's component is above zero: nothing opposes the pull.
		case mx+allow < 0 && mx-allow > -1:
			// Every point's component is below zero, and the patch's own
			// maximum is proven above -1, so this is a genuine opposition
			// rather than opposesPull's exactly-antiparallel carve-out.
			faces = append(faces, f)
		default:
			return undercutOutcome{}
		}
	}
	return undercutOutcome{faces: faces, ok: true}
}

// capPatchNormalRange is one patch's exact normal-component range against
// the unit pull p, read off its own Face.NormalAt (which already carries the
// correct outward sign): a Plane's is a single value; a Cone's (regular or
// apex) is A*cos(phi)+B*sin(phi)+C in the azimuth phi = theta - th0 measured
// from the window's own start, recovered EXACTLY from three NormalAt
// evaluations at phi = 0, pi/2, pi — f(0)=A+C, f(pi/2)=B+C, f(pi)=-A+C solves
// uniquely for A, B, C — then read with the closed-form trigRange over
// [0, th1-th0], the window IN THAT SAME azimuth. Reading the recovered local
// coefficients over the global [th0, th1] instead scans the wrong arc of the
// circle whenever th0 is not zero, and reports a range the patch never takes.
func capPatchNormalRange(f *Face, pl prismPayload, g capPatchGeom, p r3.Vec) (float64, float64, bool) {
	if !g.circular {
		n, err := f.NormalAt(pl.point(g.sideA.U, g.sideA.V, g.sideZ))
		if err != nil {
			return 0, 0, false
		}
		v := n.Value.Dot(p)
		return v, v, true
	}
	// A regular Cone patch's normal is independent of position along its own
	// ruling (azimuth alone determines it), so sampling at the cap radius —
	// which an apex patch's own zero side radius forces anyway — serves both.
	r := g.capRadius
	sampleAt := func(theta float64) (float64, bool) {
		sin, cos := math.Sincos(theta)
		pt := pl.point(g.cU+r*cos, g.cV+r*sin, g.capZ)
		n, err := f.NormalAt(pt)
		if err != nil {
			return 0, false
		}
		return n.Value.Dot(p), true
	}
	f0, ok0 := sampleAt(g.th0)
	f90, ok90 := sampleAt(g.th0 + math.Pi/2)
	f180, ok180 := sampleAt(g.th0 + math.Pi)
	if !ok0 || !ok90 || !ok180 {
		return 0, 0, false
	}
	c := (f0 + f180) / 2
	a := f0 - c
	b := f90 - c
	mn, mx := trigRange(a, b, 0, g.th1-g.th0)
	return mn + c, mx + c, true
}

// capBlendMinRadius is the tightest concave principal radius over a
// cap-blend body (Table DX, DX8).
//
// It answers only for a band whose every patch's own surface IS the Plane or
// Cone it publishes. A MITERED circular patch is not: the build rules it
// between two differently-swept directrices (docs/modify-reach-design.md
// §8.3), and a straight-ruled surface between two skewed arcs is not
// developable at all — it carries curvature in both principal directions,
// tightening as the corner's own rulings converge, and neither the Cone
// argument below nor the receiver's own section says anything about it. That
// band is UNDECIDED here (`Suspect`, through runSurveys' own refusal
// diagnostic) rather than answered with a proven absence the patch set does
// not support.
//
// For every other band this slice's patches are Plane and Cone only — a
// chamfer produces no rolling-ball surface, so there is no Torus or Sphere
// case here at all — and NEITHER kind ever tightens the answer beyond what the
// receiver's own unchanged section already gives:
//
//   - a Plane patch has zero curvature in both principal directions (flat),
//     so it contributes no radius at all, exactly as a straight prism wall
//     never does (prismMinRadius only ever reads circular walks);
//   - a regular Cone patch's ruling direction is a straight line (zero
//     curvature there too); its azimuthal principal radius is
//     R(z)/cos(halfAngle) >= R(z), so its tightest point (at the SIDE
//     boundary, R = the original wall's own radius) is never smaller than
//     that same wall's own unchanged radius — and SX7's band-reach gate
//     (buildCapBlend) guarantees a chamfered loop always keeps a strictly
//     positive unchanged run of that same wall, which prismMinRadius reads
//     directly off the untouched RECEIVER profile;
//   - an apex-cone patch's radius shrinks to exactly zero only at its own
//     boundary VERTEX (the untouched original corner point) — a sharp
//     corner/edge feature, which this survey's own convention already
//     excludes everywhere else ("the survey reads faces' principal radii,
//     not edges", shell_cup.go's cupMinRadius) — so reporting it here would
//     single out a reflex corner's un-rounded tip for a reading the SAME
//     corner, unchamfered, never received either.
//
// So for a band of those patches the correct answer is exactly "no new
// concave principal radius" and the whole survey reduces to prismMinRadius on
// the receiver's own untouched profile.
func capBlendMinRadius(cbp capBlendPayload) (radiusOutcome, bool) {
	for _, patch := range cbp.patches {
		if capPatchWindowSkew(patch.geom) > 0 {
			return radiusOutcome{}, false
		}
	}
	return prismMinRadius(prismPayload{profile: cbp.profile})
}
