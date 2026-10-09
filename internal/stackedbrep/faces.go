package stackedbrep

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/r3"
)

// Level is one held stack level and its axial displacement.
type Level struct {
	Held, Delta float64
}

// FaceRecord is one assembled BRep face before the root payload validates it.
type FaceRecord struct {
	Frame            r3.Frame
	Region           *brepgeom.Profile
	Outward          bool
	Sweep            r3.Vec
	Wall             CurveSegment
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
	Side0, Side1     []brepgeom.Split
	Delta            float64
}

// pieceKey names one swept wall piece by carrier, ends and walk direction.
type pieceKey struct {
	c        Carrier
	from, to Point2
	ccw      bool
	closed   bool
}

// sweptFaces joins identical curved or oblique wall pieces in adjacent slabs.
// A body vertex at the intermediate level splits that side line.
func (b *Engine) sweptFaces(levels []Level, ref r3.Frame, delta float64) ([]FaceRecord, error) {
	type run struct {
		seg      CurveSegment
		from, to Point2
		closed   bool
		k0, k1   int
		splits   [2][]brepgeom.Split
	}
	var runs []*run
	byKey := map[pieceKey]*run{}
	for k, loops := range b.SlabLoops {
		for _, loop := range loops {
			for _, u := range loop.Units {
				if u.Carrier.Kind == PlaneCarrier {
					continue
				}
				segs, err := b.UnitSegments(u, loop.Closed, k)
				if err != nil {
					return nil, err
				}
				for _, seg := range segs {
					w, err := boundarywalk.WalkOf(seg, nil)
					if err != nil {
						return nil, err
					}
					from, to := Point2{U: w.StartU + 0, V: w.StartV + 0}, Point2{U: w.EndU + 0, V: w.EndV + 0}
					key := pieceKey{c: u.Carrier, from: from, to: to, ccw: u.CCW, closed: w.Closed}
					if r, ok := byKey[key]; ok && r.k1 == k-1 {
						if !w.Closed {
							for side, p := range [2]Point2{from, to} {
								if b.HasEvent(p, k) {
									r.splits[side] = append(r.splits[side], brepgeom.Split{Z: levels[k].Held, ZDelta: levels[k].Delta})
								}
							}
						}
						r.k1 = k
						continue
					}
					r := &run{seg: seg, from: from, to: to, closed: w.Closed, k0: k, k1: k}
					runs = append(runs, r)
					byKey[key] = r
				}
			}
		}
	}
	out := make([]FaceRecord, 0, len(runs))
	for _, r := range runs {
		lo, hi := levels[r.k0], levels[r.k1+1]
		out = append(out, FaceRecord{Frame: ref, Wall: r.seg, Z0: lo.Held, Z1: hi.Held,
			Z0Delta: lo.Delta, Z1Delta: hi.Delta, Side0: r.splits[0], Side1: r.splits[1], Delta: delta})
	}
	return out, nil
}

// wallFaces joins the surviving rectangle edges on each axis-aligned wall plane.
func (b *Engine) wallFaces(levels []Level, ref r3.Frame, delta float64) ([]FaceRecord, error) {
	pieces := map[brepgeom.StackedWallKey]map[brepgeom.StackedWallSegment]struct{}{}
	var order []brepgeom.StackedWallKey
	add := func(key brepgeom.StackedWallKey, from, to [3]float64) error {
		set, ok := pieces[key]
		if !ok {
			set = map[brepgeom.StackedWallSegment]struct{}{}
			pieces[key] = set
			order = append(order, key)
		}
		if _, dup := set[brepgeom.StackedWallSegment{From: from, To: to}]; dup {
			return brepgeom.ErrStackedWallMiss
		}
		if _, rev := set[brepgeom.StackedWallSegment{From: to, To: from}]; rev {
			delete(set, brepgeom.StackedWallSegment{From: to, To: from})
			return nil
		}
		set[brepgeom.StackedWallSegment{From: from, To: to}] = struct{}{}
		return nil
	}
	for k, loops := range b.SlabLoops {
		z0, z1 := levels[k].Held, levels[k+1].Held
		for _, loop := range loops {
			for _, u := range loop.Units {
				if u.Carrier.Kind != PlaneCarrier {
					continue
				}
				key := brepgeom.StackedWallKey{Axis: u.Carrier.Axis, Level: u.Carrier.Level}
				if u.Carrier.Axis == 0 {
					key.Sign = 1
					if u.To.V < u.From.V {
						key.Sign = -1
					}
				} else {
					key.Sign = -1
					if u.To.U < u.From.U {
						key.Sign = 1
					}
				}
				at := func(p Point2, z float64) [3]float64 { return [3]float64{p.U + 0, p.V + 0, z} }
				bottom := append([]Point2{u.From}, b.CutsOnLine(u, k)...)
				bottom = append(bottom, u.To)
				for i := 0; i+1 < len(bottom); i++ {
					if err := add(key, at(bottom[i], z0), at(bottom[i+1], z0)); err != nil {
						return nil, err
					}
				}
				if err := add(key, at(u.To, z0), at(u.To, z1)); err != nil {
					return nil, err
				}
				top := append([]Point2{u.From}, b.CutsOnLine(u, k+1)...)
				top = append(top, u.To)
				for i := len(top) - 1; i > 0; i-- {
					if err := add(key, at(top[i], z1), at(top[i-1], z1)); err != nil {
						return nil, err
					}
				}
				if err := add(key, at(u.From, z1), at(u.From, z0)); err != nil {
					return nil, err
				}
			}
		}
	}
	var out []FaceRecord
	for _, key := range order {
		frame, embed, err := brepgeom.StackedWallFrame(ref, key)
		if err != nil {
			return nil, err
		}
		loops, err := brepgeom.ChainStackedWall(pieces[key], b.LevelAt, b.Events)
		if err != nil {
			return nil, err
		}
		regions, level, err := brepgeom.StackedWallRegions(embed, loops)
		if err != nil {
			return nil, err
		}
		for _, wallRegion := range regions {
			region := wallRegion
			out = append(out, FaceRecord{Frame: frame, Region: &region, Outward: true,
				Sweep: ref.N(), Z0: level, Z1: level, Delta: delta})
		}
	}
	return out, nil
}

// AssembleFaces records the horizontal caps, curved pieces and planar walls.
// The caller validates planar regions and checks BRep closure.
func (b *Engine) AssembleFaces(levels []Level, ref r3.Frame, delta float64) ([]FaceRecord, error) {
	var out []FaceRecord
	for _, f := range b.Faces {
		region, err := b.FaceRecord(f)
		if err != nil {
			return nil, err
		}
		l := levels[f.Level]
		out = append(out, FaceRecord{Frame: ref, Region: &region, Outward: f.Outward,
			Z0: l.Held, Z1: l.Held, Z0Delta: l.Delta, Z1Delta: l.Delta, Delta: delta})
	}
	swept, err := b.sweptFaces(levels, ref, delta)
	if err != nil {
		return nil, err
	}
	out = append(out, swept...)
	walls, err := b.wallFaces(levels, ref, delta)
	if err != nil {
		return nil, err
	}
	return append(out, walls...), nil
}
