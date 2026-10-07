package linkagebound

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ReachSource supplies the root linkage's joint order, exact frames, and
// rest boxes. RestBox returns false when any body box cannot be read exactly.
// Reach must return a fresh rational: the whole-drive sum multiplies it by
// a radius. Reach is read when the bound walk visits a joint.
type ReachSource interface {
	JointCount() int
	Parent(int) int
	Revolute(int) bool
	Frame(int) motionbound.MotionFrame
	RestBox(int) (motionbound.RatVec, motionbound.RatVec, bool)
	Reach(int) *big.Rat
}

// ReachBound is one link's path, axis radii, and whole-drive travel bound.
type ReachBound struct {
	Path  []int
	Rho   []*big.Rat
	Reach *big.Rat
}

// reachCylinder encloses a link around an exact axis line.
type reachCylinder struct {
	point, dir motionbound.RatVec
	r          *big.Rat
}

// reachBall encloses a link under the joints already visited.
type reachBall struct {
	c motionbound.RatVec
	r *big.Rat
}

// distanceSq is |x − c|², exactly.
func distanceSq(x, c motionbound.RatVec) *big.Rat {
	sum := new(big.Rat)
	for i := range 3 {
		d := new(big.Rat).Sub(x[i], c[i])
		sum.Add(sum, d.Mul(d, d))
	}
	return sum
}

// lineDistanceSq is |(x − c) × a|²/|a|², exactly.
func lineDistanceSq(x, c, a motionbound.RatVec, aSq *big.Rat) *big.Rat {
	var w motionbound.RatVec
	for i := range 3 {
		w[i] = new(big.Rat).Sub(x[i], c[i])
	}
	cx := new(big.Rat).Sub(proofbound.RatMul(w[1], a[2]), proofbound.RatMul(w[2], a[1]))
	cy := new(big.Rat).Sub(proofbound.RatMul(w[2], a[0]), proofbound.RatMul(w[0], a[2]))
	cz := new(big.Rat).Sub(proofbound.RatMul(w[0], a[1]), proofbound.RatMul(w[1], a[0]))
	d := proofbound.RatAdd(proofbound.RatMul(cx, cx), proofbound.RatMul(cy, cy), proofbound.RatMul(cz, cz))
	return d.Quo(d, aSq)
}

// BoxCorners enumerates the eight corners of an exact box in bit order.
func BoxCorners(lo, hi motionbound.RatVec) [8]motionbound.RatVec {
	var out [8]motionbound.RatVec
	for corner := range 8 {
		for i := range 3 {
			out[corner][i] = lo[i]
			if corner&(1<<i) != 0 {
				out[corner][i] = hi[i]
			}
		}
	}
	return out
}

func reachCross(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(proofbound.RatMul(a[1], b[2]), proofbound.RatMul(a[2], b[1])),
		new(big.Rat).Sub(proofbound.RatMul(a[2], b[0]), proofbound.RatMul(a[0], b[2])),
		new(big.Rat).Sub(proofbound.RatMul(a[0], b[1]), proofbound.RatMul(a[1], b[0])),
	}
}

// ReadReachBounds reads each link's path, ρ_{ik} at its revolute joints, and
// its whole-drive reach (docs/linkage-check-design.md §5.2). It walks down
// the path from the link toward ground, carrying a ball under the joints
// visited so far. A box corner's greatest axis distance gives the link's
// own revolute radius. A revolute joint above reads dist(ball centre, axis)
// plus the ball radius. Its new ball has that joint's centre and radius
// |old centre − joint centre| plus the old radius. A prismatic joint grows
// the ball radius by its greatest drive displacement.
//
// Beside the ball, the first revolute joint starts an infinite cylinder
// around its axis. A parallel revolute joint can read the distance between
// the two axes plus the old cylinder radius; the smaller of that and the
// ball's reading is ρ_{ik}. A nonparallel revolute joint restarts the
// cylinder. A prismatic joint preserves a parallel cylinder and otherwise
// grows its radius by its greatest displacement. The whole-drive reach uses
// the ball readings. Every square root rounds up and is read back as a
// rational before any addition or multiplication.
//
// A box or root that cannot be read returns ok false.
func ReadReachBounds(src ReachSource) ([]ReachBound, bool) {
	bounds := make([]ReachBound, src.JointCount())
	for k := range bounds {
		var path []int
		for at := k; at >= 0; at = src.Parent(at) {
			path = append([]int{at}, path...)
		}
		lo, hi, ok := src.RestBox(k)
		if !ok {
			return nil, false
		}
		rho := make([]*big.Rat, len(path))
		ball, own, ok := ownJointBall(src, k, lo, hi)
		if !ok {
			return nil, false
		}
		rho[len(path)-1] = own
		var cyl *reachCylinder
		if src.Revolute(k) {
			f := src.Frame(k)
			cyl = &reachCylinder{point: f.Center, dir: f.Axis, r: own}
		}
		ballRho := slices.Clone(rho)
		for n := len(path) - 2; n >= 0; n-- {
			i := path[n]
			f := src.Frame(i)
			if !src.Revolute(i) {
				ball = reachBall{c: ball.c, r: new(big.Rat).Add(ball.r, src.Reach(i))}
				if cyl != nil && !ratZero(reachCross(cyl.dir, f.Axis)) {
					cyl = &reachCylinder{point: cyl.point, dir: cyl.dir, r: new(big.Rat).Add(cyl.r, src.Reach(i))}
				}
				continue
			}
			d := sqrtUpRat(lineDistanceSq(ball.c, f.Center, f.Axis, axisSq(f.Axis)))
			toCenter := sqrtUpRat(distanceSq(ball.c, f.Center))
			if d == nil || toCenter == nil {
				return nil, false
			}
			ballRho[n] = new(big.Rat).Add(d, ball.r)
			rho[n] = ballRho[n]
			ball = reachBall{c: f.Center, r: toCenter.Add(toCenter, ball.r)}
			switch {
			case cyl == nil:
				cyl = &reachCylinder{point: f.Center, dir: f.Axis, r: rho[n]}
			case ratZero(reachCross(cyl.dir, f.Axis)):
				between := sqrtUpRat(lineDistanceSq(cyl.point, f.Center, f.Axis, axisSq(f.Axis)))
				if between == nil {
					return nil, false
				}
				if viaCyl := between.Add(between, cyl.r); viaCyl.Cmp(rho[n]) < 0 {
					rho[n] = viaCyl
				}
				cyl = &reachCylinder{point: f.Center, dir: f.Axis, r: rho[n]}
			default:
				cyl = &reachCylinder{point: f.Center, dir: f.Axis, r: rho[n]}
			}
		}
		reach := new(big.Rat)
		for n, i := range path {
			m := src.Reach(i)
			if ballRho[n] != nil {
				m.Mul(m, ballRho[n])
			}
			reach.Add(reach, m)
		}
		bounds[k] = ReachBound{Path: path, Rho: rho, Reach: reach}
	}
	return bounds, true
}

// ownJointBall reads one link's box under its own joint. A revolute joint
// publishes its largest corner-to-axis distance and a ball centred on the
// joint. A prismatic joint centres the ball on the box and grows its
// half-diagonal by the joint's reach.
func ownJointBall(src ReachSource, k int, lo, hi motionbound.RatVec) (reachBall, *big.Rat, bool) {
	f := src.Frame(k)
	corners := BoxCorners(lo, hi)
	if !src.Revolute(k) {
		var c, half motionbound.RatVec
		for i := range 3 {
			c[i] = new(big.Rat).Add(lo[i], hi[i])
			c[i].Quo(c[i], big.NewRat(2, 1))
			half[i] = new(big.Rat).Sub(hi[i], c[i])
		}
		r := sqrtUpRat(distanceSq(half, motionbound.RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}))
		if r == nil {
			return reachBall{}, nil, false
		}
		return reachBall{c: c, r: r.Add(r, src.Reach(k))}, nil, true
	}
	aSq := axisSq(f.Axis)
	axisFar, centreFar := new(big.Rat), new(big.Rat)
	for _, x := range corners {
		if d := lineDistanceSq(x, f.Center, f.Axis, aSq); d.Cmp(axisFar) > 0 {
			axisFar = d
		}
		if d := distanceSq(x, f.Center); d.Cmp(centreFar) > 0 {
			centreFar = d
		}
	}
	rho, r := sqrtUpRat(axisFar), sqrtUpRat(centreFar)
	if rho == nil || r == nil {
		return reachBall{}, nil, false
	}
	return reachBall{c: f.Center, r: r}, rho, true
}
