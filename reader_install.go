package decad

import (
	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/decad/internal/payloadclass"
)

// init installs the reader export's STEP writer calls to keep a face whose
// tag departs from its surface out of the analytic arm
// (agent-docs/step-export-design.md). It lives in the root because it reads
// the unexported node a Face handle holds.
func init() {
	facedeparture.Reader = func(face any) bool {
		f, ok := face.(*Face)
		if !ok || f == nil {
			return true
		}
		return facedeparture.TagDeparts(&f.n)
	}
}

// init installs the reader decadtest.PayloadClass calls. Tests need it because
// a Boolean result that keeps its operands' surfaces and curves
// (agent-docs/faceted-source-retention-design.md) no longer reveals the mesh path
// through any public reading.
func init() {
	payloadclass.Reader = func(body any) string {
		b, ok := body.(*Body)
		if !ok || b == nil {
			return ""
		}
		return payloadclass.TypeName(b.n.Payload)
	}
}
