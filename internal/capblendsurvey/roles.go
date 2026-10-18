package capblendsurvey

import (
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// facesByRole selects the last face with each role from the body's own step.
// It walks nodes in the same order as Body.Faces and root facesByRole.
func facesByRole(b *topo.Body) map[string]*topo.Face {
	faces := topobuild.Faces(b)
	faceRoles := make([][]string, len(faces))
	for i, f := range faces {
		for _, o := range f.Origins {
			if o.Producer == b.Origin.Producer {
				faceRoles[i] = append(faceRoles[i], o.Role)
			}
		}
	}
	roles := make(map[string]*topo.Face)
	for role, i := range selectorquery.RoleIndex(faceRoles) {
		roles[role] = faces[i]
	}
	return roles
}

// listVerdict keeps a proven opposing face beside any undecided face.
func listVerdict(faces *[]*topo.Face, undecided *bool, f *topo.Face, verdict survey2d.PullVerdict, ok bool) bool {
	if !ok {
		return false
	}
	switch verdict {
	case survey2d.PullOpposes:
		*faces = append(*faces, f)
	case survey2d.PullUndecided:
		*undecided = true
	}
	return true
}
