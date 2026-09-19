"""Resolve explicit relative actor references without scene-specific layouts."""


def ordered_npcs(npcs):
    by_id = {n['id']: n for n in npcs}
    if len(by_id) != len(npcs) or 'ego' in by_id:
        raise ValueError('NPC actor ids must be unique and distinct from ego')
    result, visiting, visited = [], set(), set()
    def visit(npc):
        nid = npc['id']
        if nid in visiting:
            raise ValueError('relative_to actor references contain a cycle')
        if nid in visited:
            return
        visiting.add(nid)
        reference = npc.get('relative_to', 'ego')
        if reference != 'ego':
            if reference not in by_id:
                raise ValueError('relative_to must name a declared actor')
            visit(by_id[reference])
        visiting.remove(nid); visited.add(nid); result.append(npc)
    for npc in npcs:
        visit(npc)
    return result
