"""Validated localization XYZ from either owned, committed map-bundle format.

No dependency on quadruped_navigation: that package already depends on this one.
Localization verifies the whole bundle's file integrity and the localization
payload semantics; OctoMap/layer semantics remain the navigation loader's job.
"""
from pathlib import Path
import numpy as np
import yaml
from map_bundle import identifier, load_bundle, read_pcd, sha256, validate_rotation


def load_localization_source(directory):
    directory = Path(directory).resolve()
    manifest = yaml.safe_load((directory / 'manifest.yaml').read_text())
    if manifest.get('kind') == 'localization_pcd':
        manifest, points = load_bundle(directory)
    elif manifest.get('kind') == 'navigation_map_bundle':
        if manifest.get('schema_version') != 1 or manifest.get('status') != 'complete' or manifest.get('frame_id') != 'map':
            raise ValueError('Incomplete navigation map')
        identifier(manifest['map_id'])
        files = manifest['files_sha256']
        required = {'localization.pcd', 'navigation.pcd', 'source_manifest.yaml',
                    'meta.yaml', 'octomap_msg.npz', 'layers.npz', 'configuration/map_tools.yaml'}
        if not required.issubset(files):
            raise ValueError('Incomplete navigation bundle file inventory')
        for relative, expected in files.items():
            path = (directory / relative).resolve()
            if not path.is_relative_to(directory) or not path.is_file() or sha256(path) != expected:
                raise ValueError('Missing or changed bundle file: ' + relative)
        if files['localization.pcd'] != manifest['parent_localization_sha256'] or files['source_manifest.yaml'] != manifest['parent_manifest_sha256']:
            raise ValueError('Parent map hashes differ')
        transform = np.asarray(manifest['navigation_from_parent'])
        if transform.shape != (4, 4) or not np.allclose(transform, np.eye(4), atol=1e-12, rtol=0):
            raise ValueError('Localization requires the preserved parent-map frame')
        source = yaml.safe_load((directory / 'source_manifest.yaml').read_text())
        if (source.get('kind') != 'localization_pcd' or source.get('status') != 'complete'
                or source.get('schema_version') != 1 or source.get('frame_id') != 'map'
                or source['files_sha256']['localization.pcd'] != files['localization.pcd']
                or source['map_from_camera_init'] != manifest['map_from_camera_init']):
            raise ValueError('Invalid parent localization contract')
        validate_rotation(source['map_from_camera_init']['rotation'])
        translation = np.asarray(source['map_from_camera_init']['translation_m'])
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError('Invalid historical map reference transform')
        points = read_pcd(directory / 'localization.pcd')
        if (len(points) != source['point_count']
                or not np.allclose(points[:, :3].min(axis=0), source['bounds']['min'], atol=1e-6)
                or not np.allclose(points[:, :3].max(axis=0), source['bounds']['max'], atol=1e-6)):
            raise ValueError('Localization payload differs from parent manifest')
    else:
        raise ValueError('Expected a committed localization or navigation map bundle')
    return dict(map_id=manifest['map_id'], kind=manifest['kind'], frame_id='map',
                directory=str(directory), point_count=len(points),
                localization_sha256=sha256(directory / 'localization.pcd')), points[:, :3].astype(np.float64)
