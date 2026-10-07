package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ScenePlane holds the exact axes and length enclosures of one loop scene.
type ScenePlane struct {
	Frame      r3.Frame
	U, V       motionbound.RatVec
	ULen, VLen proofbound.RatInterval
}

// SceneFrame constructs a loop scene's plane from its held axis and optional primary slide.
func SceneFrame(normal motionbound.RatVec, coord int, slideDir *r3.Vec, mirror, halfTurn bool) (ScenePlane, error) {
	n := normal
	if mirror {
		n = NegVec(n)
	}
	slideAxis := -1
	if slideDir != nil {
		if idx, _, ok := CoordinateAxis(*slideDir); ok {
			slideAxis = idx
		}
	}
	var uf, vf r3.Vec
	switch {
	case coord >= 0 && slideDir == nil:
		sense := vecSign(normal, coord)
		if mirror {
			sense = -sense
		}
		axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
		uf, vf = axes[(coord+1)%3], axes[(coord+2)%3]
		if sense < 0 {
			vf = vf.Scale(-1)
		}
	case coord >= 0 && slideAxis >= 0:
		sense := vecSign(normal, coord)
		if mirror {
			sense = -sense
		}
		_, dir, _ := CoordinateAxis(*slideDir)
		uf = UnitAxis(slideAxis, dir)
		vf = UnitAxis(coord, sense).Cross(uf)
	case slideDir != nil:
		uf = *slideDir
		vf = VecFloat(n).Cross(uf)
	default:
		m := 0
		for i := 1; i < 3; i++ {
			if new(big.Rat).Abs(normal[i]).Cmp(new(big.Rat).Abs(normal[m])) < 0 {
				m = i
			}
		}
		e := [3]float64{}
		e[m] = 1
		uf = VecFloat(normal).Cross(r3.NewVec(e[0], e[1], e[2]))
		vf = VecFloat(n).Cross(uf)
	}
	if halfTurn {
		uf, vf = uf.Scale(-1), vf.Scale(-1)
	}
	frame, err := r3.NewFrame(r3.Vec{}, uf, vf)
	if err != nil {
		return ScenePlane{}, err
	}
	// uf holds u exactly: a unit axis, the slide's direction, or n × e_m,
	// whose components are 0 and ± n's own. v is formed exactly from it.
	u := ExactVec(uf)
	v := Cross(n, u)
	return ScenePlane{Frame: frame, U: u, V: v, ULen: vecNorm(u), VLen: vecNorm(v)}, nil
}

// PlaneCoordinates encloses a point's exact coordinates in the plane.
func PlaneCoordinates(p, u, v motionbound.RatVec, uLen, vLen proofbound.RatInterval) (proofbound.RatInterval, proofbound.RatInterval) {
	return ratQuoInterval(Dot(p, u), uLen), ratQuoInterval(Dot(p, v), vLen)
}

func ratQuoInterval(num *big.Rat, den proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := new(big.Rat).Quo(num, den.Hi), new(big.Rat).Quo(num, den.Lo)
	if num.Sign() < 0 {
		lo, hi = hi, lo
	}
	return proofbound.IntervalOwned(lo, hi)
}

func vecNorm(w motionbound.RatVec) proofbound.RatInterval {
	sq := Dot(w, w)
	return proofbound.IntervalOwned(proofarith.FloatRat(proofbound.RatSqrtDown(sq)), proofarith.FloatRat(proofbound.RatSqrtUp(sq)))
}

// CoordinateAxis reads an exactly coordinate-aligned vector's axis and sense.
func CoordinateAxis(v r3.Vec) (int, int, bool) {
	c := [3]float64{v.X, v.Y, v.Z}
	idx := -1
	for i, x := range c {
		if x == 0 {
			continue
		}
		if idx >= 0 {
			return 0, 0, false
		}
		idx = i
	}
	if idx < 0 {
		return 0, 0, false
	}
	if c[idx] < 0 {
		return idx, -1, true
	}
	return idx, 1, true
}

// UnitAxis is the coordinate axis with the given sense.
func UnitAxis(axis, sense int) r3.Vec {
	c := [3]float64{}
	c[axis] = float64(sense)
	return r3.NewVec(c[0], c[1], c[2])
}

// ExactVec reads a finite held vector exactly.
func ExactVec(v r3.Vec) motionbound.RatVec {
	out, _ := motionbound.RatVecOf(v)
	return out
}

// NegVec negates each exact vector component.
func NegVec(w motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{new(big.Rat).Neg(w[0]), new(big.Rat).Neg(w[1]), new(big.Rat).Neg(w[2])}
}

func vecSign(w motionbound.RatVec, i int) int {
	if w[i].Sign() < 0 {
		return -1
	}
	return 1
}

// VecFloat returns the float vector nearest w.
func VecFloat(w motionbound.RatVec) r3.Vec {
	x, _ := w[0].Float64()
	y, _ := w[1].Float64()
	z, _ := w[2].Float64()
	return r3.NewVec(x, y, z)
}

// Dot forms an exact vector dot product in coordinate order.
func Dot(a, b motionbound.RatVec) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

// Cross forms an exact vector cross product in coordinate order.
func Cross(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(proofbound.RatMul(a[1], b[2]), proofbound.RatMul(a[2], b[1])),
		new(big.Rat).Sub(proofbound.RatMul(a[2], b[0]), proofbound.RatMul(a[0], b[2])),
		new(big.Rat).Sub(proofbound.RatMul(a[0], b[1]), proofbound.RatMul(a[1], b[0])),
	}
}
