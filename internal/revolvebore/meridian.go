package revolvebore

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Meridian is an exact axis-coordinate envelope of a linear meridian.
type Meridian struct {
	Low, High, MinRadius  float64
	LowRadius, HighRadius float64
	LowCapSegment         int
	HighCapSegment        int
	HasSlopedWall         bool
}

// LinearMeridian proves one axis run, two radial ends, and one strictly
// monotone outer run. It refuses axis-coordinate conversions that round.
func LinearMeridian(rp revolveplan.Payload) (Meridian, bool) {
	segments := rp.Profile.Outer.Segments
	if len(segments) < 4 {
		return Meridian{}, false
	}
	type point struct{ z, r float64 }
	points := make([]point, len(segments))
	var previous sectionrecord.Point2
	var first sectionrecord.Point2
	for i, segment := range segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return Meridian{}, false
		}
		start, end := line.Start, line.End
		if line.TStart == 1 {
			start, end = end, start
		}
		if i == 0 {
			first = start
		} else if start != previous {
			return Meridian{}, false
		}
		previous = end
		z, r, ok := exactAxisPoint(rp, start)
		if !ok || r < 0 {
			return Meridian{}, false
		}
		points[i] = point{z, r}
	}
	if previous != first {
		return Meridian{}, false
	}
	axis, radial, direction := 0, 0, 0
	var result Meridian
	for i, a := range points {
		b := points[(i+1)%len(points)]
		switch {
		case a.r == 0 && b.r == 0 && a.z != b.z:
			axis++
			result.Low, result.High = min(a.z, b.z), max(a.z, b.z)
		case a.z == b.z && (a.r == 0) != (b.r == 0):
			radial++
		case a.r > 0 && b.r > 0 && a.z == b.z && a.r != b.r:
			if result.MinRadius == 0 {
				result.MinRadius = min(a.r, b.r)
			} else {
				result.MinRadius = min(result.MinRadius, a.r, b.r)
			}
		case a.r > 0 && b.r > 0 && a.z != b.z:
			if a.r != b.r {
				result.HasSlopedWall = true
			}
			sign := 1
			if b.z < a.z {
				sign = -1
			}
			if direction != 0 && direction != sign {
				return Meridian{}, false
			}
			direction = sign
			if result.MinRadius == 0 {
				result.MinRadius = min(a.r, b.r)
			} else {
				result.MinRadius = min(result.MinRadius, a.r, b.r)
			}
		default:
			return Meridian{}, false
		}
	}
	if axis != 1 || radial != 2 || direction == 0 || !(result.Low < result.High) || result.MinRadius <= rp.Ax.SnapTol {
		return Meridian{}, false
	}
	for _, p := range points {
		if p.z < result.Low || p.z > result.High || p.r == 0 && p.z != result.Low && p.z != result.High {
			return Meridian{}, false
		}
		if p.r > 0 && p.z == result.Low {
			result.LowRadius = p.r
		}
		if p.r > 0 && p.z == result.High {
			result.HighRadius = p.r
		}
	}
	for i, a := range points {
		b := points[(i+1)%len(points)]
		if a.z != b.z || (a.r == 0) == (b.r == 0) {
			continue
		}
		switch a.z {
		case result.Low:
			result.LowCapSegment = i
		case result.High:
			result.HighCapSegment = i
		}
	}
	return result, result.LowRadius > 0 && result.HighRadius > 0
}

func exactAxisPoint(rp revolveplan.Payload, p sectionrecord.Point2) (float64, float64, bool) {
	rat := proofarith.FloatRat
	u, v, au, av := rat(p.U), rat(p.V), rat(rp.Ax.AU), rat(rp.Ax.AV)
	du, dv := rat(rp.Ax.DU), rat(rp.Ax.DV)
	if u == nil || v == nil || au == nil || av == nil || du == nil || dv == nil {
		return 0, 0, false
	}
	x, y := new(big.Rat).Sub(u, au), new(big.Rat).Sub(v, av)
	z := new(big.Rat).Add(new(big.Rat).Mul(x, du), new(big.Rat).Mul(y, dv))
	r := new(big.Rat).Sub(new(big.Rat).Mul(y, du), new(big.Rat).Mul(x, dv))
	zf, _ := z.Float64()
	rf, _ := r.Float64()
	return zf, rf, proofarith.RationalFloatError(z, zf) == 0 && proofarith.RationalFloatError(r, rf) == 0
}
