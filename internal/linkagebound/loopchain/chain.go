package loopchain

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	subs         linkagebound.DriverSegments
	readingFloor int64
	valueAt      func(side int, s *big.Rat) (float64, float64)
}

// NewChain records the drive partition and its exact scene-value reader.
func NewChain(subs linkagebound.DriverSegments, readingFloor int64,
	valueAt func(side int, s *big.Rat) (float64, float64)) *Chain {
	return &Chain{
		Asks: make(map[string]*LocatedAsk), subs: subs,
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
