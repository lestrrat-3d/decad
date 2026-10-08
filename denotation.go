package decad

import (
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/r3"
)

// The document owns the counters; internal/denotation owns the certificates.
type levelID = denotation.LevelID
type levelToken = denotation.LevelToken
type curveID = denotation.CurveID
type curveToken = denotation.CurveToken

func (d *Document) mintLevel(origin, normal r3.Vec) levelToken {
	d.nextLevel++
	return denotation.NewLevel(d.nextLevel, origin, normal)
}

func sameLevel(a, b levelToken) bool { return denotation.SameLevel(a, b) }

func (d *Document) mintCurve() curveToken {
	d.nextCurve++
	return denotation.NewCurve(d.nextCurve)
}

func sameCurve(a, b curveToken) bool { return denotation.SameCurve(a, b) }
