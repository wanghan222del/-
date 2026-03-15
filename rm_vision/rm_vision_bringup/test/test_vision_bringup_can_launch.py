from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def load_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'vision_bringup_can.launch.py'
    spec = spec_from_file_location('vision_bringup_can_launch', launch_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_vision_bringup_can_uses_multithreaded_container():
    module = load_launch_module()
    launch_description = module.generate_launch_description()

    container = None
    for entity in launch_description.entities:
        if entity.__class__.__name__ != 'ComposableNodeContainer':
            continue
        if getattr(entity, '_Node__node_name', None) == 'camera_detector_container':
            container = entity
            break

    assert container is not None, 'camera_detector_container not found'
    assert getattr(container, '_Node__node_executable', None) == 'component_container_mt'
