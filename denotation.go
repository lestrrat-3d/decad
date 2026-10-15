package decad

import (
	"github.com/lestrrat-3d/decad/internal/denotation"
)

// The document node owns the counters and mints through them
// (internal/topo Document.MintLevel, MintCurve); internal/denotation owns the
// certificates.
type levelID = denotation.LevelID
type levelToken = denotation.LevelToken
type curveToken = denotation.CurveToken

func sameLevel(a, b levelToken) bool { return denotation.SameLevel(a, b) }
