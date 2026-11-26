import { align, refine } from './align.js';

self.onmessage = ({ data: { mode, target, source, up } }) => {
    try {
        self.postMessage((mode === 'refine' ? refine : align)(target, source, up));
    } catch (e) {
        self.postMessage({ error: e.message });
    }
};
