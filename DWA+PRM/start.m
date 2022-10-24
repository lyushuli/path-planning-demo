% start = [1, 1]; tar = [9, 9, 0.5];
% start = [1, 1]; tar = [2, 9, 0.5];
% start = [1, 5]; tar = [9, 5, 0.5]
% start = [9, 3]; tar = [1, 7, 0.5];

while true

main([1, 1, rand * 2 * pi], [9, 9, 0.5]);
pause(1);

main([1, 9, rand * 2 * pi], [9, 1, 0.5]);
pause(1);

main([1, 5, rand * 2 * pi], [9, 5, 0.5]);
pause(1);

main([5, 1, rand * 2 * pi], [5, 9, 0.5]);
pause(1);
end