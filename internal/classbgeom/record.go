package classbgeom

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// NaturalRecord reports whether every segment is a line, circle, or arc
// recorded over its natural range, walked in either direction.
func NaturalRecord(loops []sectionrecord.LoopRecord) bool {
	natural := func(t0, t1 float64) bool { return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0) }
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			switch s := seg.(type) {
			case sectionrecord.LineSeg:
				if !natural(s.TStart, s.TEnd) {
					return false
				}
			case sectionrecord.ArcSeg:
				if !natural(s.TStart, s.TEnd) {
					return false
				}
			case sectionrecord.CircleSeg:
				if !natural(s.TStart, s.TEnd) {
					return false
				}
			default:
				return false
			}
		}
	}
	return true
}

// ShiftedRecord moves a tool's record to its axes at the reference origin.
// Every coordinate uses the exact rational dot of the origins' difference
// with an axis. ok is false when a moved coordinate or level rounds.
func ShiftedRecord(ref, from, to r3.Frame, loops []sectionrecord.LoopRecord, z0, z1 float64) (
	[]sectionrecord.LoopRecord, float64, float64, bool) {
	ox, oy := ref.Origin(), from.Origin()
	diff := [3]*big.Rat{
		new(big.Rat).Sub(proofarith.FloatRat(oy.X), proofarith.FloatRat(ox.X)),
		new(big.Rat).Sub(proofarith.FloatRat(oy.Y), proofarith.FloatRat(ox.Y)),
		new(big.Rat).Sub(proofarith.FloatRat(oy.Z), proofarith.FloatRat(ox.Z)),
	}
	var shift [3]*big.Rat
	for i, axis := range [3]r3.Vec{to.U(), to.V(), to.N()} {
		shift[i] = proofbound.RatAdd(
			proofbound.RatMul(diff[0], proofarith.FloatRat(axis.X)),
			proofbound.RatMul(diff[1], proofarith.FloatRat(axis.Y)),
			proofbound.RatMul(diff[2], proofarith.FloatRat(axis.Z)),
		)
	}
	ok := true
	move := func(value float64, i int) float64 {
		exact := new(big.Rat).Add(proofarith.FloatRat(value), shift[i])
		held, _ := exact.Float64()
		if proofarith.RationalFloatError(exact, held) != 0 {
			ok = false
		}
		return held
	}
	point := func(p sectionrecord.Point2) sectionrecord.Point2 {
		return sectionrecord.Point2{U: move(p.U, 0), V: move(p.V, 1)}
	}
	moved := make([]sectionrecord.LoopRecord, len(loops))
	for li, loop := range loops {
		for _, seg := range loop.Segments {
			switch s := seg.(type) {
			case sectionrecord.LineSeg:
				s.Start, s.End = point(s.Start), point(s.End)
				seg = s
			case sectionrecord.ArcSeg:
				s.Center, s.Start, s.End = point(s.Center), point(s.Start), point(s.End)
				seg = s
			case sectionrecord.CircleSeg:
				s.Center = point(s.Center)
				seg = s
			}
			moved[li].Segments = append(moved[li].Segments, seg)
		}
	}
	return moved, move(z0, 2), move(z1, 2), ok
}

// AxisAligned reports whether every line of the record follows one local
// coordinate axis. Circular segments have no line direction and pass.
func AxisAligned(loops []sectionrecord.LoopRecord) bool {
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			line, ok := seg.(sectionrecord.LineSeg)
			if !ok {
				continue
			}
			du, dv := line.End.U-line.Start.U, line.End.V-line.Start.V
			if du != 0 && dv != 0 {
				return false
			}
		}
	}
	return true
}
