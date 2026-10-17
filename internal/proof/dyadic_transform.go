package proof

import "github.com/lestrrat-3d/r3"

// DyadicMap is DvTransform with the transform's translation and basis lifted
// once for a caller mapping many points through one pose.
type DyadicMap struct {
	translation, ex, ey, ez DyV3
}

// NewDyadicMap lifts a transform's translation and basis once.
func NewDyadicMap(t r3.Transform) DyadicMap {
	b := t.Basis()
	return DyadicMap{translation: DyVec(t.Translation()),
		ex: DyVec(b.EX), ey: DyVec(b.EY), ez: DyVec(b.EZ)}
}

// Apply returns exactly what DvTransform returns for the same point.
func (m DyadicMap) Apply(p DyV3) DyV3 {
	return DvAdd(m.translation, DvAdd(DvScale(m.ex, p[0]),
		DvAdd(DvScale(m.ey, p[1]), DvScale(m.ez, p[2]))))
}
