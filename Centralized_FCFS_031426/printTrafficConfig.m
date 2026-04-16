function printTrafficConfig(config, filename)
% Print config to command window.
% If filename is provided, also save it to a text .m file.
%
% Usage:
%   printTrafficConfig(config)
%   printTrafficConfig(config, 'saved_config.m')

    doSave = (nargin >= 2) && ~isempty(filename);

    if doSave
        fid = fopen(filename, 'w');
        if fid == -1
            error('Cannot open file: %s', filename);
        end
    end

    localPrint(sprintf('config = {\n'));
    for i = 1:numel(config)
        e = config{i}.entrance;
        exits = config{i}.exits;

        line1 = sprintf('    struct(''entrance'', %d, ''exits'', [', e);
        localPrint(line1);

        for j = 1:numel(exits)
            token = sprintf('%d ', exits(j));
            localPrint(token);
        end

        localPrint(sprintf(']),\n'));
    end
    localPrint(sprintf('};\n'));

    if doSave
        fclose(fid);
        fprintf('Config saved to %s\n', filename);
    end

    function localPrint(str)
        fprintf('%s', str);
        if doSave
            fprintf(fid, '%s', str);
        end
    end
end