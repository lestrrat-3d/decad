package motionrun

import (
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// StaticPairState records which whole-path gate first settles a moving and
// stationary body. The caller keeps pair ordering and publication.
type StaticPairState uint8

const (
	StaticPairEvaluated StaticPairState = iota
	StaticPairInvalid
	StaticPairExcluded
	StaticPairSheet
)

// ClassifyStaticPair checks validity before swept-box exclusion and sheet
// status, preserving the order used by the shared motion/linkage engine.
func ClassifyStaticPair(mover, stationary *topo.Body, moverValid, stationaryValid bool,
	sweptLo, sweptHi motionbound.RatVec, sweptOK bool) (StaticPairState, float64) {
	if !moverValid || !stationaryValid {
		return StaticPairInvalid, 0
	}
	if sweptOK {
		if lower, ok := motionbound.SweptBoxLower(sweptLo, sweptHi, stationary.Bounds); ok {
			return StaticPairExcluded, lower
		}
	}
	if mover.Kind == topo.BodySheet || stationary.Kind == topo.BodySheet {
		return StaticPairSheet, 0
	}
	return StaticPairEvaluated, 0
}
