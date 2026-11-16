package prismrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// CarrierRecord is a displaced section's per-segment and per-junction proof
// (agent-docs/carrier-record-design.md §3). Segment[l][j] is ε, the
// displacement of segment j of loop l's carrier: the line through its
// recorded Start and End, or the circle its recorded centre and radius state
// (an ArcSeg's radius is |Start − Center|). Junction[l][j] is δ, the
// displacement of the exact point where segment j of loop l ends and segment
// j+1 starts. Loop 0 is the outer loop and loop i+1 is hole i.
//
// A valid record proves that the section the payload denotes has the
// recorded section's loops and segments in order, of the same kinds and walk
// senses (CR1), on carriers within ε (CR2), with every junction within δ
// (CR3), and every entry at most SectionDelta (CR4).
//
// SectionDelta and Profile are the payload's own when the record was made. A
// copy that changes either leaves the record stale, and Payload.CarrierRecord
// then answers false: a stale record withholds a claim and never states one.
type CarrierRecord struct {
	SectionDelta float64
	Profile      momentinput.Profile
	Segment      [][]float64
	Junction     [][]float64

	// owner is the one body the record was minted for (BuildWithRecord). A
	// copy of the payload in another body, or a payload copied out of its body,
	// names another owner or none and claims nothing (§3.4).
	owner *topo.Body
}

// CarrierRecord returns the payload's carrier record when it is valid. A
// payload with no section displacement denotes its own record and answers a
// record of zeros. A displaced payload answers its record only when the
// record's SectionDelta is bit-identical to the payload's, its profile equals
// the payload's loop for loop, and its entries match the profile's shape and
// are finite, non-negative and at most SectionDelta.
func (pp Payload) CarrierRecord(budget *proofbound.WorkBudget) (CarrierRecord, bool, error) {
	if pp.SectionDelta == 0 {
		return ZeroCarrierRecord(pp.Profile), true, nil
	}
	cr := pp.Carriers
	if cr == nil || math.Float64bits(cr.SectionDelta) != math.Float64bits(pp.SectionDelta) {
		return CarrierRecord{}, false, nil
	}
	if !cr.wellFormed(pp.SectionDelta) {
		return CarrierRecord{}, false, nil
	}
	// CR4: every loop's Lemma H bound is finite and at most SectionDelta.
	for l := range cr.Segment {
		if h := cr.LoopH(l); !(h <= pp.SectionDelta) {
			return CarrierRecord{}, false, nil
		}
	}
	same, err := momentinput.ExactProfileEqual(budget, cr.Profile, pp.Profile)
	if err != nil || !same {
		return CarrierRecord{}, false, err
	}
	return *cr, true, nil
}

// ZeroCarrierRecord is the record of a section that denotes itself: every
// entry zero.
func ZeroCarrierRecord(profile momentinput.Profile) CarrierRecord {
	loops := carrierLoops(profile)
	cr := CarrierRecord{Profile: cloneProfile(profile), Segment: make([][]float64, len(loops)), Junction: make([][]float64, len(loops))}
	for l, loop := range loops {
		cr.Segment[l] = make([]float64, len(loop.Segments))
		cr.Junction[l] = make([]float64, len(loop.Segments))
	}
	return cr
}

// UniformCarrierRecord is a record whose every entry is delta, for a section
// whose every carrier datum and junction moved by at most delta.
func UniformCarrierRecord(profile momentinput.Profile, delta float64) CarrierRecord {
	cr := ZeroCarrierRecord(profile)
	cr.SectionDelta = delta
	for l := range cr.Segment {
		for j := range cr.Segment[l] {
			cr.Segment[l][j] = delta
			cr.Junction[l][j] = delta
		}
	}
	return cr
}

// Max is the largest entry of the record, zero for an empty one.
func (cr CarrierRecord) Max() float64 {
	worst := 0.0
	for l := range cr.Segment {
		worst = math.Max(worst, cr.LoopMax(l))
	}
	return worst
}

// LoopMax is the largest entry of loop l.
func (cr CarrierRecord) LoopMax(l int) float64 {
	worst := 0.0
	if l < 0 || l >= len(cr.Segment) {
		return worst
	}
	for j := range cr.Segment[l] {
		worst = math.Max(worst, math.Max(cr.Segment[l][j], cr.Junction[l][j]))
	}
	return worst
}

// ExactLoop reports whether every entry of loop l is zero: the denoted
// section holds that loop of the recorded section unchanged, point for point.
func (cr CarrierRecord) ExactLoop(l int) bool {
	if l < 0 || l >= len(cr.Segment) {
		return false
	}
	return cr.LoopMax(l) == 0
}

// wellFormed reports whether the record's entries match its profile's shape
// and are finite, non-negative and at most delta.
func (cr CarrierRecord) wellFormed(delta float64) bool {
	loops := carrierLoops(cr.Profile)
	if len(cr.Segment) != len(loops) || len(cr.Junction) != len(loops) {
		return false
	}
	for l, loop := range loops {
		if len(cr.Segment[l]) != len(loop.Segments) || len(cr.Junction[l]) != len(loop.Segments) {
			return false
		}
		for j := range loop.Segments {
			for _, v := range []float64{cr.Segment[l][j], cr.Junction[l][j]} {
				if proofbound.IsNonFinite(v) || v < 0 || v > delta {
					return false
				}
			}
		}
	}
	return true
}

// cloneProfile is a deep copy of a profile, so a record shares no segment
// slice with the payload that carries it (§3.1).
func cloneProfile(p momentinput.Profile) momentinput.Profile {
	out := momentinput.Profile{Outer: sectionrecord.CloneLoopRecord(p.Outer)}
	if p.Holes != nil {
		out.Holes = make([]sectionrecord.LoopRecord, 0, len(p.Holes)) // nil-ness is part of the profile's identity
	}
	for _, h := range p.Holes {
		out.Holes = append(out.Holes, sectionrecord.CloneLoopRecord(h))
	}
	return out
}

func carrierLoops(p momentinput.Profile) []sectionrecord.LoopRecord {
	return append([]sectionrecord.LoopRecord{p.Outer}, p.Holes...)
}

// Minted reports whether the record is freshly made by a producer and bound
// to no body yet. A record bound to a body that reaches another payload was
// passed through.
func (cr CarrierRecord) Minted() bool { return cr.owner == nil }

// BuildWithRecord is Build for a producer of §5: the new body carries a deep
// copy of rec bound to that body. Build itself always clears the record.
func BuildWithRecord(ctx context.Context, env topo.Env, pp Payload, rec CarrierRecord) (*topo.Body, error) {
	body, err := Build(ctx, env, pp, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	built, ok := body.Payload.(Payload)
	if !ok {
		return body, nil
	}
	bound := rec.Clone()
	bound.owner = body
	built.Carriers = &bound
	body.Payload = built
	return body, nil
}

// OwnedPayload is body's payload with a record it does not own removed: a
// record bound to another body (a payload copied between bodies) claims
// nothing. Every reader of a record starts from it or CarrierRecordOf.
func OwnedPayload(body *topo.Body) topo.Payload {
	pp, ok := body.Payload.(Payload)
	if !ok {
		return body.Payload
	}
	if pp.Carriers != nil && pp.Carriers.owner != body {
		pp.Carriers = nil
	}
	return pp
}

// CarrierRecordOf is the body's carrier record when it is valid and owned by
// the body (§3.4): a payload with no displacement answers a record of zeros.
func CarrierRecordOf(budget *proofbound.WorkBudget, body *topo.Body) (CarrierRecord, bool, error) {
	pp, ok := OwnedPayload(body).(Payload)
	if !ok {
		return CarrierRecord{}, false, nil
	}
	return pp.CarrierRecord(budget)
}
