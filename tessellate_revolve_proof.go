package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the proof half of docs/tessellation-design.md §13's increment T2
// (docs/tessellation-reach-design.md §6, R3): everything the revolve
// tessellator must PROVE about the mesh it assembles, kept apart from the
// assembly itself in tessellate_revolve.go.
//
// It owns four things:
//
//   - §8's two coordinate stages. deltaC is the displacement from every IDEAL
//     unplaced sample — the exact evaluation of X(z, ρ, φ) on the payload's own
//     floats — to the binary64 vertex this build stored for it, enclosed with
//     rational intervals throughout and never with a library trig call. deltaR
//     is the displacement from the exact rigid image of that stored unplaced
//     vertex to the final placed one.
//   - §8's tolerance split, which reserves both of them before a single chord
//     is chosen and refuses when nothing is left.
//   - §9's endpoint and homotopy audits, discharged in ONE pass over the final
//     stored triangles (tessellation.RevolveContactAudit's own doc comment carries the
//     derivation).
//   - §10.2's Ecell, the cut-stable area allowance of one wall cell, in closed
//     form by complete sign decomposition (tessellation.RevolveCellAreaSlack).
//
// Nothing here samples anything to decide anything. Every enclosure is a
// proofbound.RatInterval built from the payload's held float64s, which are exact
// rationals, so the answers do not move with the platform's libm or with FMA
// contraction.

// revolveAngularSequence builds the angular sequence for n chords, over the
// payload's own denotation (docs/evaluator-design.md §6): sample l's angle is
// enclosed as enc(phi0) + (l/n)·(enc(phi1) − enc(phi0)), so the stored
// cosine/sine is checked against the angle the RECORD denotes, not merely the
// held float the resolver rounded to. Wherever the payload's denotation
// cannot state an end exactly (den.phi0/den.phi1 invalid — a ToFaceAngular
// stop, a payload literal with none, or an angle unit this evaluator does not
// denote), this falls back to the prior reading over the held floats alone,
// which reproduces today's construction exactly: a partial sweep's angle
// φ0 + l·(φ1 − φ0)/n as an exact rational in the payload's own two floats
// (survey2d.RadSinCosInterval), a full turn starting at zero as l/n of a TURN
// (proofbound.TurnSinCosInterval, no π entering at all), and a full turn starting
// elsewhere as the same radian enclosure over φ0 + 2π·l/n, widened by the 2π
// enclosure's own (sub-2⁻²⁴⁰) width.
func revolveAngularSequence(rp revolvePayload, n int) (tessellation.RevolveAngular, error) {
	if n <= 0 {
		return tessellation.RevolveAngular{}, fmt.Errorf(`%w: a revolve mesh needs at least one angular chord`, ErrDegenerate)
	}
	phi0, phi1, full := rp.phi0, rp.phi1, rp.full
	r0, r1 := proofarith.FloatRat(phi0), proofarith.FloatRat(phi1)
	if r0 == nil || r1 == nil {
		return tessellation.RevolveAngular{}, fmt.Errorf(`%w: the sweep interval is not finite, so no angular sample can be enclosed`, ErrUnsupported)
	}
	enc0, ok0 := rp.den.phi0.enclosure()
	enc1, ok1 := rp.den.phi1.enclosure()
	haveDen := ok0 && ok1
	var diff proofbound.RatInterval
	if haveDen {
		diff = proofbound.IntervalSub(enc1, enc0)
	}
	out := tessellation.RevolveAngular{N: n, Samples: n + 1}
	if full {
		out.Samples = n
	}
	nRat := new(big.Rat).SetInt64(int64(n))
	if full {
		out.Step = proofbound.IntervalScale(proofbound.TwoPiInterval(), new(big.Rat).Inv(nRat))
	} else {
		out.Step = proofbound.PointInterval(new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), nRat))
	}
	for l := range out.Samples {
		frac := new(big.Rat).SetFrac64(int64(l), int64(n))
		var cosIv, sinIv proofbound.RatInterval
		switch {
		case haveDen:
			angle := proofbound.IntervalAdd(enc0, proofbound.IntervalScale(diff, frac))
			var ok bool
			sinIv, cosIv, ok = tessellation.RadSinCosSpan(angle)
			if !ok {
				return tessellation.RevolveAngular{}, tessellation.ErrRevolveAngleEnclosure
			}
		case full && r0.Sign() == 0:
			sinIv, cosIv = proofbound.TurnSinCosInterval(frac)
		case full:
			angle := proofbound.IntervalAdd(proofbound.PointInterval(r0), proofbound.IntervalScale(proofbound.TwoPiInterval(), frac))
			var ok bool
			sinIv, cosIv, ok = tessellation.RadSinCosSpan(angle)
			if !ok {
				return tessellation.RevolveAngular{}, tessellation.ErrRevolveAngleEnclosure
			}
			// A full turn's last interval closes onto its first sample, so the
			// sequence never states φ1 and no seam ring is emitted.
		default:
			angle := new(big.Rat).Add(r0, new(big.Rat).Mul(frac, new(big.Rat).Sub(r1, r0)))
			var ok bool
			sinIv, cosIv, ok = survey2d.RadSinCosInterval(angle)
			if !ok {
				return tessellation.RevolveAngular{}, tessellation.ErrRevolveAngleEnclosure
			}
		}
		cosHeld, _ := intervalMid(cosIv).Float64()
		sinHeld, _ := intervalMid(sinIv).Float64()
		if proofbound.IsNonFinite(cosHeld) || proofbound.IsNonFinite(sinHeld) {
			return tessellation.RevolveAngular{}, tessellation.ErrRevolveAngleEnclosure
		}
		gap := math.Max(proofbound.IntervalFloatError(cosIv, cosHeld), proofbound.IntervalFloatError(sinIv, sinHeld))
		if proofbound.IsNonFinite(gap) || gap > tessellation.RevolveTrigGapPrior {
			return tessellation.RevolveAngular{}, fmt.Errorf(`%w: an angular sample's stored cosine and sine sit farther from the angle they denote than this mesh reserved for them`, ErrUnsupported)
		}
		out.Cos = append(out.Cos, cosHeld)
		out.Sin = append(out.Sin, sinHeld)
		out.CosIv = append(out.CosIv, cosIv)
		out.SinIv = append(out.SinIv, sinIv)
		out.Gap = math.Max(out.Gap, gap)
	}
	return out, nil
}

// revolveIdealBasis is docs/tessellation-design.md §8's axis basis as the EXACT
// expression the payload's own floats denote, rather than the float64 triple
// the build stores for it: a3 = O + aU·U + aV·V, w = dU·U + dV·V,
// e0 = −dV·U + dU·V and e1 = w × e0. The gap between this and the stored basis
// is one of the terms deltaC measures.
func revolveIdealBasis(rp revolvePayload) (tessellation.RevolveBasis3Iv, bool) {
	origin, ok0 := survey2d.IvVec3Of(rp.frame.Origin())
	fu, ok1 := survey2d.IvVec3Of(rp.frame.U())
	fv, ok2 := survey2d.IvVec3Of(rp.frame.V())
	aU, aV := proofarith.FloatRat(rp.ax.aU), proofarith.FloatRat(rp.ax.aV)
	dU, dV := proofarith.FloatRat(rp.ax.dU), proofarith.FloatRat(rp.ax.dV)
	if !ok0 || !ok1 || !ok2 || aU == nil || aV == nil || dU == nil || dV == nil {
		return tessellation.RevolveBasis3Iv{}, false
	}
	scale := func(v survey2d.IvVec3, s *big.Rat) survey2d.IvVec3 {
		return survey2d.IvVec3Mul(v, proofbound.PointInterval(s))
	}
	a3 := survey2d.IvVec3Add(origin, survey2d.IvVec3Add(scale(fu, aU), scale(fv, aV)))
	w := survey2d.IvVec3Add(scale(fu, dU), scale(fv, dV))
	e0 := survey2d.IvVec3Add(scale(fu, new(big.Rat).Neg(dV)), scale(fv, dU))
	return tessellation.RevolveBasis3Iv{A3: a3, W: w, E0: e0, E1: tessellation.IvVec3Cross(w, e0)}, true
}

// requireVertexLinks is docs/tessellation-design.md §9's construction safety
// net: the combinatorial link of every stored vertex — the edge each incident
// triangle contributes between its other two corners — must be ONE connected
// cycle with every vertex of degree two. A pinched pole passes the
// directed-edge audit and fails here, which is the whole reason the link audit
// exists beside it.
func requireVertexLinks(ctx context.Context, m *Mesh) error {
	budget := proofbound.NewWorkBudget(ctx)
	links := make(map[int]map[int][]int, len(m.vertices))
	add := func(center, from, to int) {
		l, ok := links[center]
		if !ok {
			l = map[int][]int{}
			links[center] = l
		}
		l[from] = append(l[from], to)
		l[to] = append(l[to], from)
	}
	for _, tri := range m.triangles {
		if err := budget.Step(); err != nil {
			return err
		}
		add(tri[0], tri[1], tri[2])
		add(tri[1], tri[2], tri[0])
		add(tri[2], tri[0], tri[1])
	}
	// Vertex index order, never map order: a refusal names the FIRST vertex
	// that fails, so two runs over the same mesh report the same one.
	for center := range m.vertices {
		link, ok := links[center]
		if !ok {
			continue
		}
		if err := budget.Step(); err != nil {
			return err
		}
		start := -1
		for v, nbrs := range link {
			if len(nbrs) != 2 {
				return fmt.Errorf(`%w: the mesh vertex at index %d has a pinched link: its neighbour %d meets %d link edges rather than two`, ErrUnsupported, center, v, len(nbrs))
			}
			if start < 0 || v < start {
				start = v
			}
		}
		if start < 0 {
			continue
		}
		seen := map[int]struct{}{start: {}}
		prev, cur := -1, start
		for {
			nbrs := link[cur]
			next := nbrs[0]
			if next == prev {
				next = nbrs[1]
			}
			if next == start {
				break
			}
			if _, done := seen[next]; done {
				return fmt.Errorf(`%w: the mesh vertex at index %d has a pinched link`, ErrUnsupported, center)
			}
			seen[next] = struct{}{}
			prev, cur = cur, next
		}
		if len(seen) != len(link) {
			return fmt.Errorf(`%w: the mesh vertex at index %d has a link of %d cycles rather than one`, ErrUnsupported, center, 1+len(link)-len(seen))
		}
	}
	return budget.Err()
}
