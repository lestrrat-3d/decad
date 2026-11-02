package prismplacement

import "github.com/lestrrat-3d/decad/internal/denotation"

// LevelDenotationSnapshot binds the original exact pointer and its contents.
type LevelDenotationSnapshot = denotation.LevelDenotationSnapshot

// LevelDenotationSnapshotCost bounds the snapshot's rational copy.
func LevelDenotationSnapshotCost(d LevelDenotation) (uint64, bool) {
	return denotation.LevelDenotationSnapshotCost(d)
}

// SnapshotLevelDenotation captures the original level identity.
func SnapshotLevelDenotation(d LevelDenotation) LevelDenotationSnapshot {
	return denotation.SnapshotLevelDenotation(d)
}
