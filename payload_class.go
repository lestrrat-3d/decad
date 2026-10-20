package decad

import (
	"github.com/lestrrat-3d/decad/internal/payloadclass"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
)

// init installs the reader decadtest.PayloadClass calls. Tests need it because
// a Boolean result that keeps its operands' surfaces and curves
// (docs/faceted-source-retention-design.md) no longer reveals the mesh path
// through any public reading.
func init() {
	payloadclass.Reader = func(body any) string {
		b, ok := body.(*Body)
		if !ok || b == nil {
			return ""
		}
		return reportvocab.PayloadTypeName(b.n.Payload)
	}
}
