package loopchain

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// LocatedAsk remembers the scene that certified an enclosure or refusal.
type LocatedAsk struct {
	Scene Scene
	Ask
}

// Chain owns the canonical, cached enclosure asks for a loop drive. Its caller
// serializes access because the underlying sketch is shared by all asks.
type Chain struct {
	scenes       [2]Scene
	zero         [2]*LocatedAsk
	Asks         map[string]*LocatedAsk
	spans        map[string][][]linkagebound.Span
	subs         linkagebound.DriverSegments
	readingFloor int64
	valueAt      func(side int, s *big.Rat) (float64, float64)
}

// NewChain records the drive partition and its exact scene-value reader.
func NewChain(subs linkagebound.DriverSegments, readingFloor int64,
	valueAt func(side int, s *big.Rat) (float64, float64)) *Chain {
	return &Chain{
		Asks: make(map[string]*LocatedAsk), spans: make(map[string][][]linkagebound.Span), subs: subs,
		readingFloor: readingFloor, valueAt: valueAt,
	}
}

// SetScene installs a side's certified zero pose before any asks on that side.
func (c *Chain) SetScene(side int, scene Scene, zero Ask) {
	c.scenes[side] = scene
	c.zero[side] = &LocatedAsk{Scene: scene, Ask: zero}
}

// Value reads dependent j from the ask's scene and whole-turn continuation.
func (c *Chain) Value(ask *LocatedAsk, j int) (motionbound.MotionParam, motionbound.MotionParam) {
	return Value(ask.Scene, ask.Ask, j)
}

// Hull encloses dependent j over every supplied ask.
func (c *Chain) Hull(asks []*LocatedAsk, j int) proofbound.RatInterval {
	ivs := make([]proofbound.RatInterval, len(asks))
	for n, ask := range asks {
		ivs[n] = linkagebound.ValueInterval(c.Value(ask, j))
	}
	return linkagebound.Hull(ivs...)
}

// enclose continues from pred or reuses a certified ask at the same key.
func (c *Chain) enclose(ctx context.Context, key string, lo, hi float64, pred *LocatedAsk) (*LocatedAsk, error) {
	if ask, ok := c.Asks[key]; ok {
		return ask, nil
	}
	ask, err := Continue(ctx, pred.Scene, lo, hi, pred.Ask)
	if err != nil {
		return nil, err
	}
	result := &LocatedAsk{Scene: pred.Scene, Ask: ask}
	c.Asks[key] = result
	return result, nil
}

// Point asks at s by continuing from its canonical predecessor.
func (c *Chain) Point(ctx context.Context, sub linkagebound.DriverSubsegment, s *big.Rat) (*LocatedAsk, error) {
	if sub.Held {
		s = sub.Near
	}
	if s.Cmp(sub.Near) == 0 {
		if sub.NearZero {
			return c.zero[sub.Side], nil
		}
		approach, err := c.Approach(ctx, sub)
		if err != nil {
			return nil, err
		}
		lo, hi := c.valueAt(sub.Side, s)
		return c.enclose(ctx, linkagebound.AskKey(sub.Idx, "p", s), lo, hi, approach)
	}
	key := linkagebound.AskKey(sub.Idx, "p", s)
	if ask, ok := c.Asks[key]; ok {
		return ask, nil
	}
	start := linkagebound.ChainStart(sub.Lo, sub.Near, s, c.readingFloor)
	cell, err := c.Cell(ctx, sub, start, s)
	if err != nil {
		return nil, err
	}
	lo, hi := c.valueAt(sub.Side, s)
	return c.enclose(ctx, key, lo, hi, cell)
}

// StraddleAsks returns the approaches and cut points of a straddle's neighbours.
func (c *Chain) StraddleAsks(ctx context.Context, sub linkagebound.DriverSubsegment) ([]*LocatedAsk, error) {
	var out []*LocatedAsk
	for _, nb := range []linkagebound.DriverSubsegment{c.subs[sub.Idx-1], c.subs[sub.Idx+1]} {
		approach, err := c.Approach(ctx, nb)
		if err != nil {
			return nil, err
		}
		at, err := c.Point(ctx, nb, nb.Near)
		if err != nil {
			return nil, err
		}
		out = append(out, approach, at)
	}
	return out, nil
}

// PointValues reads every dependent at s from its canonical chain. A value
// beyond its certified reach is refused with the caller's loop-specific error.
func (c *Chain) PointValues(ctx context.Context, s *big.Rat, dependents int, reach []*big.Rat,
	refuse func(error) error) ([][2]motionbound.MotionParam, error) {
	sub := c.subs.At(s)
	if sub.Straddle {
		return c.straddleValues(ctx, sub, dependents, reach, refuse)
	}
	ask, err := c.Point(ctx, sub, s)
	if err != nil {
		return nil, err
	}
	if ask.Err != nil {
		return nil, refuse(ask.Err)
	}
	out := make([][2]motionbound.MotionParam, dependents)
	for j := range dependents {
		lo, hi := c.Value(ask, j)
		if linkagebound.Magnitude(linkagebound.ValueInterval(lo, hi)).Cmp(reach[j]) > 0 {
			return nil, refuse(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))
		}
		out[j] = [2]motionbound.MotionParam{lo, hi}
	}
	return out, nil
}

// straddleValues reads each dependent's hull from the neighbouring chains.
func (c *Chain) straddleValues(ctx context.Context, sub linkagebound.DriverSubsegment,
	dependents int, reach []*big.Rat, refuse func(error) error) ([][2]motionbound.MotionParam, error) {
	asks, err := c.StraddleAsks(ctx, sub)
	if err != nil {
		return nil, err
	}
	for _, ask := range asks {
		if ask.Err != nil {
			return nil, refuse(ask.Err)
		}
	}
	out := make([][2]motionbound.MotionParam, dependents)
	for j := range dependents {
		h := c.Hull(asks, j)
		if linkagebound.Magnitude(h).Cmp(reach[j]) > 0 {
			return nil, refuse(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))
		}
		out[j] = [2]motionbound.MotionParam{{Turn: new(big.Rat), Base: h.Lo}, {Turn: new(big.Rat), Base: h.Hi}}
	}
	return out, nil
}

// Approach encloses the path from the zero pose to a subsegment's near end.
func (c *Chain) Approach(ctx context.Context, sub linkagebound.DriverSubsegment) (*LocatedAsk, error) {
	lo, _ := c.valueAt(sub.Side, sub.Near)
	_, zero := ZeroRange(c.scenes[sub.Side].Offset)
	return c.enclose(ctx, linkagebound.AskKey(sub.Idx, "a"), zero, lo, c.zero[sub.Side])
}

// Cell encloses a piece in chain order, continuing from its start point.
func (c *Chain) Cell(ctx context.Context, sub linkagebound.DriverSubsegment, start, end *big.Rat) (*LocatedAsk, error) {
	if sub.Held {
		return c.Point(ctx, sub, sub.Near)
	}
	key := linkagebound.AskKey(sub.Idx, "c", start, end)
	if ask, ok := c.Asks[key]; ok {
		return ask, nil
	}
	from, err := c.Point(ctx, sub, start)
	if err != nil {
		return nil, err
	}
	_, lo := c.valueAt(sub.Side, start)
	hi, _ := c.valueAt(sub.Side, end)
	return c.enclose(ctx, key, lo, hi, from)
}

// ChainCell orders [a, b] along its subsegment's enclosure chain.
func (c *Chain) ChainCell(ctx context.Context, sub linkagebound.DriverSubsegment, a, b *big.Rat) (*LocatedAsk, error) {
	if sub.Increasing() {
		return c.Cell(ctx, sub, a, b)
	}
	return c.Cell(ctx, sub, b, a)
}

// Decomposition is the certified parts of a drive and each dependent's reach.
type Decomposition struct {
	Certified [][2]*big.Rat
	Hulls     []proofbound.RatInterval
	Reach     []*big.Rat
}

// Decompose asks a drive as cells and bisects refused cells down to floor.
// The caller serializes access to the chain and checks dependent joint limits.
func (c *Chain) Decompose(ctx context.Context, dependents int, floor *big.Rat) (Decomposition, error) {
	var result Decomposition
	add := func(ask *LocatedAsk) {
		if ask.Err != nil {
			return
		}
		for j := range dependents {
			iv := linkagebound.ValueInterval(c.Value(ask, j))
			if len(result.Hulls) <= j {
				result.Hulls = append(result.Hulls, iv)
				continue
			}
			result.Hulls[j] = linkagebound.Hull(result.Hulls[j], iv)
		}
	}
	for _, sub := range c.subs {
		if sub.Straddle {
			// Its neighbours' near ends are its values' sources.
			continue
		}
		near, err := c.Point(ctx, sub, sub.Near)
		if err != nil {
			return Decomposition{}, err
		}
		add(near)
	}
	var walk func(a, b *big.Rat) error
	walk = func(a, b *big.Rat) error {
		var asks []*LocatedAsk
		refused := false
		for _, pc := range c.subs.Pieces(a, b) {
			if pc.Sub.Straddle {
				st, err := c.StraddleAsks(ctx, pc.Sub)
				if err != nil {
					return err
				}
				for _, ask := range st {
					refused = refused || ask.Err != nil
				}
				asks = append(asks, st...)
				continue
			}
			cell, err := c.ChainCell(ctx, pc.Sub, pc.Lo, pc.Hi)
			if err != nil {
				return err
			}
			refused = refused || cell.Err != nil
			asks = append(asks, cell)
			for _, end := range []*big.Rat{pc.Lo, pc.Hi} {
				point, err := c.Point(ctx, pc.Sub, end)
				if err != nil {
					return err
				}
				asks = append(asks, point)
			}
		}
		if !refused {
			for _, ask := range asks {
				add(ask)
			}
			result.Certified = append(result.Certified, [2]*big.Rat{a, b})
			return nil
		}
		width := new(big.Rat).Sub(b, a)
		if width.Cmp(floor) <= 0 {
			return nil
		}
		mid := new(big.Rat).Add(a, b)
		mid.Quo(mid, big.NewRat(2, 1))
		if err := walk(a, mid); err != nil {
			return err
		}
		return walk(mid, b)
	}
	if err := walk(new(big.Rat), big.NewRat(1, 1)); err != nil {
		return Decomposition{}, err
	}
	result.Reach = make([]*big.Rat, dependents)
	for j := range dependents {
		result.Reach[j] = new(big.Rat)
		if j < len(result.Hulls) {
			result.Reach[j] = linkagebound.Magnitude(result.Hulls[j])
		}
	}
	return result, nil
}

// IntervalSpans reads every dependent over [a, b] on each subsegment's chain.
// refuse maps a certified ask's refusal into the caller's loop-specific error.
func (c *Chain) IntervalSpans(ctx context.Context, a, b *big.Rat, dependents int,
	reach []*big.Rat, refuse func(error) error) ([][]linkagebound.Span, error) {
	var out [][]linkagebound.Span
	for _, pc := range c.subs.Pieces(a, b) {
		if pc.Sub.Straddle {
			piece, err := c.straddleSpans(ctx, pc.Sub, dependents, reach, refuse)
			if err != nil {
				return nil, err
			}
			out = append(out, piece)
			continue
		}
		pa, err := c.Point(ctx, pc.Sub, pc.Lo)
		if err != nil {
			return nil, err
		}
		pb, err := c.Point(ctx, pc.Sub, pc.Hi)
		if err != nil {
			return nil, err
		}
		cell, err := c.ChainCell(ctx, pc.Sub, pc.Lo, pc.Hi)
		if err != nil {
			return nil, err
		}
		for _, ask := range []*LocatedAsk{pa, cell, pb} {
			if ask.Err != nil {
				return nil, refuse(ask.Err)
			}
		}
		piece := make([]linkagebound.Span, dependents)
		for j := range dependents {
			A, B, C := linkagebound.ValueInterval(c.Value(pa, j)), linkagebound.ValueInterval(c.Value(pb, j)),
				linkagebound.ValueInterval(c.Value(cell, j))
			H := linkagebound.Hull(A, B, C)
			if linkagebound.Magnitude(H).Cmp(reach[j]) > 0 {
				return nil, refuse(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))
			}
			piece[j] = linkagebound.Span{Start: A, End: B, Hull: H}
		}
		out = append(out, piece)
	}
	c.spans[linkagebound.IntervalKey(a, b)] = out
	return out, nil
}

// Span returns the sum of a dependent's travel bounds over a cached interval.
// The caller serializes access to the chain and first checks the joint index.
func (c *Chain) Span(j int, a, b *big.Rat) *big.Rat {
	pieces, ok := c.cachedSpans(a, b)
	if !ok {
		return nil
	}
	return linkagebound.SumSpanUpper(pieces, j)
}

// DependentHull returns a dependent's hull over a cached interval.
func (c *Chain) DependentHull(j int, a, b *big.Rat) (proofbound.RatInterval, bool) {
	pieces, ok := c.cachedSpans(a, b)
	if !ok || len(pieces) == 0 {
		return proofbound.RatInterval{}, false
	}
	return linkagebound.HullOfPieces(pieces, j), true
}

func (c *Chain) cachedSpans(a, b *big.Rat) ([][]linkagebound.Span, bool) {
	if a.Cmp(b) > 0 {
		a, b = b, a
	}
	pieces, ok := c.spans[linkagebound.IntervalKey(a, b)]
	return pieces, ok
}

// straddleSpans reads a whole straddle from its two neighbouring chains.
func (c *Chain) straddleSpans(ctx context.Context, sub linkagebound.DriverSubsegment, dependents int,
	reach []*big.Rat, refuse func(error) error) ([]linkagebound.Span, error) {
	asks, err := c.StraddleAsks(ctx, sub)
	if err != nil {
		return nil, err
	}
	for _, ask := range asks {
		if ask.Err != nil {
			return nil, refuse(ask.Err)
		}
	}
	piece := make([]linkagebound.Span, dependents)
	for j := range dependents {
		h := c.Hull(asks, j)
		if linkagebound.Magnitude(h).Cmp(reach[j]) > 0 {
			return nil, refuse(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))
		}
		piece[j] = linkagebound.Span{Start: linkagebound.ValueInterval(c.Value(asks[1], j)),
			End: linkagebound.ValueInterval(c.Value(asks[3], j)), Hull: h}
	}
	return piece, nil
}
