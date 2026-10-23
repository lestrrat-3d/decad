package loftface

import (
	"math/big"
)

// A crossing can enter another whole Sketch entity through their shared
// endpoint twice in one LoftFaces band. These exact rational brackets cover
// every zero of the quadratic endpoint polynomial in band time. Each simple
// root changes side. The caller must bind the same live source endpoint on
// both adjacent entities before interpreting either change as a joint seam.
type EndpointRoot struct {
	Time      Interval
	X         Interval
	Direction int
	Finite    bool
}

// TimePiece is one open or closed piece of band time with one side sign.
type TimePiece struct {
	Lo, Hi         *big.Rat
	LoOpen, HiOpen bool
	Side           int
	RootIndex      int // -1 for a strict-side piece
}
