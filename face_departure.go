package decad

import (
	"github.com/lestrrat-3d/decad/internal/facedeparture"
)

// init installs the reader export's STEP writer calls to keep a face whose
// tag departs from its surface by a stated bound out of the analytic arm
// (agent-docs/step-export-design.md). It lives in the root because it reads
// the unexported node a Face handle holds.
func init() {
	facedeparture.Reader = func(face any) bool {
		f, ok := face.(*Face)
		if !ok || f == nil {
			return true
		}
		return f.n.NormalBound != 0 || f.n.HeldNormalBound != 0
	}
}
