package dynamics

import (
	"cmp"
	"math"
	"math/bits"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// This file is the direct start of the island proposal
// (docs/multibody-dynamics-design.md §6.2): an island of at most
// directPointLimit manifold points first solves its frictionless normal
// complementarity problem by active-set enumeration, and the projected
// Gauss–Seidel sweeps start from that solution. Like the sweeps, the start
// proves nothing; the certificate judges what the sweeps publish.

// directPointLimit bounds the islands the direct start enumerates: 2^8 − 1
// active sets of at most eight points.
const directPointLimit = 8

// directRankFloor is the relative eigenvalue below which a direction of an
// active set's effective-mass matrix counts as null: an indeterminate patch,
// such as a box face's four corners, then takes the minimum-norm split.
const directRankFloor = 0x1p-40

// directRow is one scalar row of the direct start: point k along dir, with
// the target post-solve relative speed along dir.
type directRow struct {
	point  int
	dir    r3.Vec
	target float64
}

// directStart solves the island's contact problem at its pre-solve velocities
// for a start of the sweeps. When a point has friction it first tries the
// sticking solution: every active point meets its normal target and stops
// sliding, with each tangent impulse inside the point's nominal Coulomb disk.
// Otherwise, or when no sticking solution exists, it solves the frictionless
// problem. It reports false when neither is accepted or the island has more
// than directPointLimit points.
func (w *World) directStart(points []nominalPoint, bodies []nominalBody) ([]float64, [][2]float64, bool) {
	if len(points) == 0 || len(points) > directPointLimit {
		return nil, nil, false
	}
	if slices.ContainsFunc(points, func(p nominalPoint) bool { return p.mu > 0 }) {
		if lambda, tangent, ok := w.directSolve(points, bodies, true); ok {
			return lambda, tangent, true
		}
	}
	return w.directSolve(points, bodies, false)
}

// directSolve enumerates the active sets from the largest to the smallest,
// ties by bit order. An active point contributes its normal row and, when
// stick is set and it has friction, its two tangent rows with target zero;
// each set solves K·λ = target − w0 by the minimum-norm pseudo-inverse. A set
// is accepted when every normal impulse is nonnegative, every sticking
// tangent impulse lies in its disk within ImpulseResidual/16, and every row, active or not, meets its
// target within VelocityResidual/16 (an inactive normal row at or above it).
func (w *World) directSolve(points []nominalPoint, bodies []nominalBody, stick bool) ([]float64, [][2]float64, bool) {
	n := len(points)
	slack := w.step.VelocityResidual.Base() / 16
	var rows []directRow
	for k, p := range points {
		rows = append(rows, directRow{point: k, dir: p.n, target: p.target})
		if stick && p.mu > 0 {
			rows = append(rows, directRow{point: k, dir: p.t1}, directRow{point: k, dir: p.t2})
		}
	}
	m := len(rows)
	relative := func(state []nominalBody, row directRow) float64 {
		p := points[row.point]
		return state[p.b].pointVelocity(p.rB).Sub(state[p.a].pointVelocity(p.rA)).Dot(row.dir)
	}
	// K[i][j] is row i's change of relative speed per unit impulse on row j.
	k := make([][]float64, m)
	for i := range k {
		k[i] = make([]float64, m)
	}
	for j, column := range rows {
		probe := make([]nominalBody, len(bodies))
		for slot, body := range bodies {
			probe[slot] = nominalBody{dynamic: body.dynamic, invMass: body.invMass, invInertia: body.invInertia}
		}
		q := points[column.point]
		probe[q.a].apply(column.dir.Scale(-1), q.rA)
		probe[q.b].apply(column.dir, q.rB)
		for i, row := range rows {
			k[i][j] = relative(probe, row)
		}
	}
	gap := make([]float64, m)
	for i, row := range rows {
		gap[i] = row.target - relative(bodies, row)
	}
	masks := make([]uint, 0, 1<<n)
	for mask := uint(1<<n) - 1; mask > 0; mask-- {
		masks = append(masks, mask)
	}
	masks = append(masks, 0)
	slices.SortStableFunc(masks, func(a, b uint) int {
		if ca, cb := bits.OnesCount(a), bits.OnesCount(b); ca != cb {
			return cb - ca
		}
		return cmp.Compare(a, b)
	})
	for _, mask := range masks {
		var active []int
		for i, row := range rows {
			if mask&(1<<row.point) != 0 {
				active = append(active, i)
			}
		}
		impulse := make([]float64, m)
		if len(active) > 0 {
			sub := make([][]float64, len(active))
			rhs := make([]float64, len(active))
			for r, i := range active {
				sub[r] = make([]float64, len(active))
				for c, j := range active {
					sub[r][c] = k[i][j]
				}
				rhs[r] = gap[i]
			}
			solved, ok := pseudoSolve(sub, rhs)
			if !ok {
				continue
			}
			for r, i := range active {
				impulse[i] = solved[r]
			}
		}
		coneSlack := w.step.ImpulseResidual.Base() / 16
		lambda, tangent, ok := directAccepted(points, rows, k, gap, impulse, mask, slack, coneSlack)
		if !ok && stick {
			lambda, tangent, ok = directAccepted(points, rows, k, gap, proportionalFriction(points, rows, impulse),
				mask, slack, coneSlack)
		}
		if ok {
			return lambda, tangent, true
		}
	}
	return nil, nil, false
}

// proportionalFriction redistributes a sticking candidate's tangent impulses
// within each group of points that share their bodies and normal (one
// manifold patch) in proportion to the points' normal impulses. The minimum-
// norm split spreads a patch's friction evenly while its normal pressure
// shifts to balance the friction torque, which can leave the lightly pressed
// corners outside their disks; the proportional split keeps the patch's net
// friction, and directAccepted rechecks every row, so the change is kept only
// when the rows still hold.
func proportionalFriction(points []nominalPoint, rows []directRow, impulse []float64) []float64 {
	out := slices.Clone(impulse)
	normalRow := make([]int, len(points))
	for i, row := range rows {
		if row.dir == points[row.point].n {
			normalRow[row.point] = i
		}
	}
	done := make([]bool, len(points))
	for k, p := range points {
		if done[k] || p.mu == 0 {
			continue
		}
		var group []int
		for j, q := range points {
			if !done[j] && q.a == p.a && q.b == p.b && q.n == p.n && q.mu > 0 {
				group, done[j] = append(group, j), true
			}
		}
		var total [2]float64
		pressure := 0.0
		for _, j := range group {
			r := normalRow[j]
			total[0] += impulse[r+1]
			total[1] += impulse[r+2]
			pressure += impulse[r]
		}
		if pressure <= 0 {
			continue
		}
		for _, j := range group {
			r := normalRow[j]
			share := impulse[r] / pressure
			out[r+1], out[r+2] = total[0]*share, total[1]*share
		}
	}
	return out
}

// directAccepted checks one candidate and returns its per-point normal and
// tangent impulses: normal impulses nonnegative, tangent impulses inside
// each nominal disk within ImpulseResidual/16 (a slide the friction stops
// exactly lies on the rim), every active row at its target within slack and every
// inactive normal row at or above it; an inactive point's tangent rows carry
// no impulse and no condition.
func directAccepted(points []nominalPoint, rows []directRow, k [][]float64, gap, impulse []float64,
	mask uint, slack, coneSlack float64) ([]float64, [][2]float64, bool) {
	lambda := make([]float64, len(points))
	tangent := make([][2]float64, len(points))
	seen := make([]int, len(points))
	for i, row := range rows {
		change := 0.0
		for j := range rows {
			change += float64(k[i][j] * impulse[j])
		}
		residual := change - gap[i] // post relative speed minus target
		active, tangentRow := mask&(1<<row.point) != 0, seen[row.point] > 0
		switch {
		case !finite(impulse[i], residual):
			return nil, nil, false
		case active && math.Abs(residual) > slack:
			return nil, nil, false
		case !active && !tangentRow && residual < -slack:
			return nil, nil, false
		}
		switch seen[row.point] {
		case 0:
			lambda[row.point] = impulse[i]
		default:
			tangent[row.point][seen[row.point]-1] = impulse[i]
		}
		seen[row.point]++
	}
	for k, p := range points {
		if lambda[k] < 0 || math.Hypot(tangent[k][0], tangent[k][1]) > float64(p.mu*lambda[k])+coneSlack {
			return nil, nil, false
		}
	}
	return lambda, tangent, true
}

// pseudoSolve returns the minimum-norm solution of the symmetric system
// m·x = b by a cyclic Jacobi eigen-decomposition, dropping eigenvalues at or
// below directRankFloor times the largest one.
func pseudoSolve(m [][]float64, b []float64) ([]float64, bool) {
	n := len(m)
	a := make([][]float64, n)
	v := make([][]float64, n)
	for i := range n {
		a[i] = slices.Clone(m[i])
		v[i] = make([]float64, n)
		v[i][i] = 1
	}
	for range 64 {
		off := 0.0
		for p := range n {
			for q := p + 1; q < n; q++ {
				off += float64(a[p][q] * a[p][q])
			}
		}
		if off == 0 {
			break
		}
		for p := range n {
			for q := p + 1; q < n; q++ {
				if a[p][q] == 0 {
					continue
				}
				theta := (a[q][q] - a[p][p]) / (2 * a[p][q])
				t := math.Copysign(1, theta) / (math.Abs(theta) + math.Sqrt(float64(theta*theta)+1))
				c := 1 / math.Sqrt(float64(t*t)+1)
				s := t * c
				for r := range n {
					arp, arq := a[r][p], a[r][q]
					a[r][p], a[r][q] = float64(c*arp)-float64(s*arq), float64(s*arp)+float64(c*arq)
				}
				for r := range n {
					apr, aqr := a[p][r], a[q][r]
					a[p][r], a[q][r] = float64(c*apr)-float64(s*aqr), float64(s*apr)+float64(c*aqr)
				}
				for r := range n {
					vrp, vrq := v[r][p], v[r][q]
					v[r][p], v[r][q] = float64(c*vrp)-float64(s*vrq), float64(s*vrp)+float64(c*vrq)
				}
			}
		}
	}
	largest := 0.0
	for i := range n {
		largest = math.Max(largest, math.Abs(a[i][i]))
	}
	if !finite(largest) || largest <= 0 {
		return nil, false
	}
	x := make([]float64, n)
	for e := range n {
		if a[e][e] <= largest*directRankFloor {
			continue
		}
		projection := 0.0
		for r := range n {
			projection += float64(v[r][e] * b[r])
		}
		projection /= a[e][e]
		for r := range n {
			x[r] += float64(v[r][e] * projection)
		}
	}
	return x, true
}
