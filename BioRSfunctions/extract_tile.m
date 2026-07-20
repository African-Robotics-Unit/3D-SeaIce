function extract_tile(pc)
%% extract_tiles.m
% Interactive single-tile extractor — top-down XY scatter plot.
%
% USAGE:
%   1. Call extract_tiles() from the command window
%   2. Move mouse to position the tile box
%   3. Left-click  → log anchor coords to console
%   4. Press '1'   → extract tile into 'tilePc' in base workspace

% =========================================================================
%% CONFIG
% =========================================================================

tileSize = 0.10;    % 10 cm in metres — adjust if your units differ
%pc       = pcCropped;

% =========================================================================
%% FIGURE — plain top-down scatter, no pcshow
% =========================================================================

xyz = double(pc.Location);
clr = double(pc.Color) / 255;  % Nx3 normalised RGB for scatter

fig = figure('Name', 'Tile extractor | move mouse | left-click = log | press 1 = extract');
ax  = axes('Parent', fig);

scatter(ax, xyz(:,1), xyz(:,2), 1, clr, 'filled');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
title(ax, 'Move box with mouse | Left-click: log position | Press 1: extract');
hold(ax, 'on');

% =========================================================================
%% INITIAL BOX — drawn at axis centre
% =========================================================================

xl = xlim(ax);
yl = ylim(ax);
anchor = [(xl(1)+xl(2))/2 - tileSize/2, ...
          (yl(1)+yl(2))/2 - tileSize/2];

hBox = drawBox(ax, anchor, tileSize);

% =========================================================================
%% CALLBACKS
% =========================================================================

set(fig, 'WindowButtonMotionFcn', @onMouseMove);
set(fig, 'WindowButtonDownFcn',   @onMouseClick);
set(fig, 'KeyPressFcn',           @onKeyPress);

% =========================================================================
%% NESTED FUNCTIONS
% =========================================================================

    function onMouseMove(~, ~)
        pt = get(ax, 'CurrentPoint');
        anchor = [pt(1,1) - tileSize/2, pt(1,2) - tileSize/2];
        updateBox(hBox, anchor, tileSize);
        drawnow limitrate;
    end

    function onMouseClick(~, ~)
        pt     = get(ax, 'CurrentPoint');
        anchor = [pt(1,1) - tileSize/2, pt(1,2) - tileSize/2];
        fprintf('Anchor: X = %.4f  Y = %.4f\n', anchor(1), anchor(2));
    end

    function onKeyPress(~, event)
        if strcmp(event.Key, '1')
            pt     = get(ax, 'CurrentPoint');
            anchor = [pt(1,1) - tileSize/2, pt(1,2) - tileSize/2];

            % Disable callbacks
            set(fig, 'WindowButtonMotionFcn', '');
            set(fig, 'WindowButtonDownFcn',   '');
            set(fig, 'KeyPressFcn',           '');

            title(ax, 'Extracting...');
            drawnow;

            % Extract
            mask = xyz(:,1) >= anchor(1)              & ...
                   xyz(:,1) <= (anchor(1) + tileSize) & ...
                   xyz(:,2) >= anchor(2)              & ...
                   xyz(:,2) <= (anchor(2) + tileSize);

            tilePc = select(pc, mask);
            assignin('base', 'tilePc', tilePc);

            fprintf('\nExtracted %d / %d points into base workspace: tilePc\n', ...
                    sum(mask), pc.Count);

            % Highlight extracted points in red
            scatter(ax, xyz(mask,1), xyz(mask,2), 2, 'r', 'filled');
            title(ax, sprintf('Done — %d points extracted into tilePc', sum(mask)));
        end
    end

end  % end extract_tiles

% =========================================================================
%% LOCAL FUNCTIONS
% =========================================================================

function h = drawBox(ax, anchor, sz)
    ox = anchor(1);
    oy = anchor(2);
    bx = [ox; ox+sz; ox+sz; ox;   ox  ];
    by = [oy; oy;    oy+sz; oy+sz; oy ];
    h  = plot(ax, bx, by, 'r-', 'LineWidth', 2);
end

function updateBox(h, anchor, sz)
    ox = anchor(1);
    oy = anchor(2);
    set(h, 'XData', [ox; ox+sz; ox+sz; ox;   ox  ], ...
           'YData', [oy; oy;    oy+sz; oy+sz; oy ]);
end