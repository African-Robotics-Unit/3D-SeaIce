function extract_tiles(pc)
%% extract_tiles.m
% Interactive multi-tile extractor — top-down XY scatter plot.
%
% USAGE:
%   1. Call extract_tiles() from the command window
%   2. Move mouse to position the tile box
%   3. Press '1'        → capture current box into tilePcs array
%   4. Move mouse again → select next tile
%   5. Press 'R'        → reset all selections and start over
%   6. All captures live in base workspace as tilePcs{1}, tilePcs{2}, ...

% =========================================================================
%% CONFIG
% =========================================================================

tileSize = 0.10;    % 10 cm in metres
%pc       = pcCropped;

% =========================================================================
%% DATA
% =========================================================================

xyz      = double(pc.Location);
clr      = double(pc.Color) / 255;   % Nx3 normalised RGB
captured = false(pc.Count, 1);       % tracks which points are already selected
tilePcs  = {};                        % accumulator — written to base workspace

% =========================================================================
%% FIGURE
% =========================================================================

fig = figure('Name', 'Tile extractor | press 1 = capture | R = reset');
ax  = axes('Parent', fig);

% Base scatter — all points in their RGB colour
hBase = scatter(ax, xyz(:,1), xyz(:,2), 1, clr, 'filled');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
hold(ax, 'on');

% Red overlay for captured points (starts empty)
hCaptured = scatter(ax, NaN, NaN, 2, 'r', 'filled');

% Moving box
xl     = xlim(ax);
yl     = ylim(ax);
anchor = [(xl(1)+xl(2))/2 - tileSize/2, ...
          (yl(1)+yl(2))/2 - tileSize/2];
hBox   = drawBox(ax, anchor, tileSize);

% Reset button
uicontrol('Parent', fig, ...
          'Style',    'pushbutton', ...
          'String',   'Reset (R)', ...
          'Units',    'normalized', ...
          'Position', [0.01 0.01 0.10 0.05], ...
          'FontSize', 10, ...
          'Callback', @onReset);

updateTitle(ax, numel(tilePcs));

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
        pt     = get(ax, 'CurrentPoint');
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
        switch lower(event.Key)
            case '1'
                captureCurrentBox();
            case 'r'
                onReset();
        end
    end

    function captureCurrentBox(~, ~)
        pt     = get(ax, 'CurrentPoint');
        anchor = [pt(1,1) - tileSize/2, pt(1,2) - tileSize/2];

        mask = xyz(:,1) >= anchor(1)              & ...
               xyz(:,1) <= (anchor(1) + tileSize) & ...
               xyz(:,2) >= anchor(2)              & ...
               xyz(:,2) <= (anchor(2) + tileSize);

        if ~any(mask)
            fprintf('No points in selection — try repositioning.\n');
            return;
        end

        % Add to accumulator
        tilePcs{end+1} = select(pc, mask);  %#ok<AGROW>
        captured = captured | mask;

        % Update red overlay
        set(hCaptured, ...
            'XData', xyz(captured, 1), ...
            'YData', xyz(captured, 2));

        % Write array to base workspace
        assignin('base', 'tilePcs', tilePcs);

        fprintf('Tile %d captured: %d points  (total captured: %d)\n', ...
                numel(tilePcs), sum(mask), sum(captured));
        updateTitle(ax, numel(tilePcs));
        drawnow;
    end

    function onReset(~, ~)
        tilePcs  = {};
        captured = false(pc.Count, 1);

        % Clear red overlay
        set(hCaptured, 'XData', NaN, 'YData', NaN);

        assignin('base', 'tilePcs', tilePcs);
        fprintf('Reset — all selections cleared.\n');
        updateTitle(ax, 0);
        drawnow;
    end

end  % end extract_tiles

% =========================================================================
%% LOCAL FUNCTIONS
% =========================================================================

function h = drawBox(ax, anchor, sz)
    ox = anchor(1);  oy = anchor(2);
    h  = plot(ax, ...
              [ox; ox+sz; ox+sz; ox;   ox  ], ...
              [oy; oy;    oy+sz; oy+sz; oy ], ...
              'r-', 'LineWidth', 2);
end

function updateBox(h, anchor, sz)
    ox = anchor(1);  oy = anchor(2);
    set(h, 'XData', [ox; ox+sz; ox+sz; ox;   ox  ], ...
           'YData', [oy; oy;    oy+sz; oy+sz; oy ]);
end

function updateTitle(ax, n)
    if n == 0
        title(ax, 'Move box | Press 1: capture tile | R: reset');
    else
        title(ax, sprintf('%d tile(s) captured — move box and press 1 for next | R: reset', n));
    end
end