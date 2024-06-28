using System;
using System.IO;

using System.Linq;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.AspNetCore.Mvc;
using System.IdentityModel.Tokens.Jwt;
using Microsoft.IdentityModel.Tokens;
using System.Security.Claims;
using System.Text;
using System.Diagnostics;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
// Learn more about configuring Swagger/OpenAPI at https://aka.ms/aspnetcore/swashbuckle

// Configure services
builder.Services.AddDbContext<FullStackContext>(options =>
    options.UseSqlite("Datasource=./data.db"));

builder.Services.AddCors(o =>
{
    // Enable CORS
    o.AddDefaultPolicy(policy =>
        policy.AllowAnyOrigin()
              .AllowAnyMethod()
              .AllowAnyHeader());
});


var app = builder.Build();

// Auto migration logic
using (var scope = app.Services.CreateScope())
{
    var dbContext = scope.ServiceProvider.GetRequiredService<FullStackContext>();
    dbContext.Database.Migrate();
}
app.UseCors();

//Endpoint User for method Get all data, Get data use ID, Create or add data user, Update data User, Delete data user
// Endpoint for user login
// Endpoint for user login
app.MapPost("/login", async (HttpContext context, FullStackContext db) =>
{
    var requestBody = await new StreamReader(context.Request.Body).ReadToEndAsync();
    var login = JsonSerializer.Deserialize<User>(requestBody, new JsonSerializerOptions
    {
        PropertyNameCaseInsensitive = true
    });

    var trimmedUsername = login.Username?.Trim().ToLower();

    var user = await db.Users.FirstOrDefaultAsync(u => u.Username != null && u.Username.Trim().ToLower() == trimmedUsername);

    if (user == null || login.Password != user.Password) // Tidak menggunakan hashing pada password
    {
        context.Response.StatusCode = 401; // Unauthorized
        await context.Response.WriteAsync("Invalid username or password");
        return;
    }

    // Generate JWT token
    var tokenHandler = new JwtSecurityTokenHandler();
    var key = Encoding.ASCII.GetBytes("your_secret_key_here"); // Change this to your secret key
    var tokenDescriptor = new SecurityTokenDescriptor
    {
        Subject = new ClaimsIdentity(new Claim[]
        {
            new Claim(ClaimTypes.NameIdentifier, user.Id.ToString()), // Assuming user has an Id property
            // You can add more claims as needed
        }),
        Expires = DateTime.UtcNow.AddHours(1), // Token expiration time
        SigningCredentials = new SigningCredentials(new SymmetricSecurityKey(key), SecurityAlgorithms.HmacSha256Signature)
    };
    var token = tokenHandler.CreateToken(tokenDescriptor);
    var tokenString = tokenHandler.WriteToken(token);

    // Serialize user data and token to send back to the client
    var responseData = new { user = user, token = tokenString };
    await context.Response.WriteAsync(JsonSerializer.Serialize(responseData));
});

// app.MapGet("/users", async (FullStackContext db) =>
app.MapGet("/users", async (FullStackContext db) => //Field in the name backend context with your name Context (FullStackContext)
    await db.Users.ToListAsync());

// Endpoint to get a single user by id
app.MapGet("/users/{id}", async (int id, FullStackContext db) =>
    await db.Users.FindAsync(id) is User user ? Results.Ok(user) : Results.NotFound());

// Endpoint to create a new user
app.MapPost("/users", async (User user, FullStackContext db) =>
{
    // Tidak ada perubahan pada bagian ini
    // Add the user to the database
    db.Users.Add(user);

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return the created user with the appropriate status code and location header
    return Results.Created($"/users/{user.Id}", user);
});



// Endpoint to update a user
app.MapPut("/users/{id}", async (int id, User updatedUser, FullStackContext db) =>
{
    var user = await db.Users.FindAsync(id);
    if (user is null) return Results.NotFound();

    user.Username = updatedUser.Username;
    user.Password = updatedUser.Password;
    user.Phone = updatedUser.Phone;

    await db.SaveChangesAsync();
    return Results.NoContent();
});

// Endpoint to delete a user
app.MapDelete("/users/{id}", async (int id, FullStackContext db) =>
{
    if (await db.Users.FindAsync(id) is User user)
    {
        db.Users.Remove(user);
        await db.SaveChangesAsync();
        return Results.Ok(user);
    }

    return Results.NotFound();
});
// Endpoint to get all robots
app.MapGet("/robots", async (FullStackContext db) =>
    await db.Robots.ToListAsync());

// Endpoint to get a single robot by id
app.MapGet("/robots/{id}", async (int id, FullStackContext db) =>
    await db.Robots.FindAsync(id) is Robot robot ? Results.Ok(robot) : Results.NotFound());

// Endpoint to create a new robot
app.MapPost("/robots", async (Robot robot, FullStackContext db) =>
{
    // Add the robot to the database
    db.Robots.Add(robot);

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return the created robot with the appropriate status code and location header
    return Results.Created($"/robots/{robot.Id}", robot);
});

// Endpoint to update a robot
app.MapPut("/robots/{id}", async (int id, Robot updatedRobot, FullStackContext db) =>
{
    var robot = await db.Robots.FindAsync(id);
    if (robot is null) return Results.NotFound();

    robot.Name = updatedRobot.Name;
    robot.Ip = updatedRobot.Ip;
    robot.Port = updatedRobot.Port;

    await db.SaveChangesAsync();
    return Results.NoContent();
});

app.MapDelete("/robots/{id}", async (int id, FullStackContext db) =>
{
    if (await db.Robots.FindAsync(id) is Robot robot)
    {
        // Hapus robot dari tabel Robot
        db.Robots.Remove(robot);
  // Hapus semua misi yang terkait dengan robot yang dihapus
      
        // Hapus semua misi yang terkait dengan robot yang dihapus
        
        // Hapus semua jejak kaki yang terkait dengan robot yang dihapus
        var footprintsToDelete = await db.Footprints.Where(f => f.Robotname == robot.Name).ToListAsync();
        db.Footprints.RemoveRange(footprintsToDelete);

        // Hapus semua kesalahan yang terkait dengan robot yang dihapus
        var errorsToDelete = await db.Errors.Where(e => e.Robotname == robot.Name).ToListAsync();
        db.Errors.RemoveRange(errorsToDelete);

        // Hapus semua aktivitas yang terkait dengan robot yang dihapus
        var activitiesToDelete = await db.Activities.Where(a => a.Robotname == robot.Name).ToListAsync();
        db.Activities.RemoveRange(activitiesToDelete);

        // Simpan perubahan ke database
        await db.SaveChangesAsync();

        // Mengembalikan respons dengan robot yang dihapus
        return Results.Ok(robot);
    }

    return Results.NotFound();
});

app.MapGet("/missions/all", async (FullStackContext db) =>
{
    IQueryable<Mission> missions = db.Missions.Include(m => m.Waypoints);
    return await missions.ToListAsync();
});


// Endpoint to get a single mission by id
app.MapGet("/missions/{id}", async (int id, FullStackContext db) =>
    await db.Missions.FindAsync(id) is Mission mission ? Results.Ok(mission) : Results.NotFound());

app.MapPost("/missions", async (Mission mission, FullStackContext db) =>
{
    mission.CreatedAt = DateTime.UtcNow;
    db.Missions.Add(mission);
    await db.SaveChangesAsync();
    return Results.Created($"/missions/{mission.Id}", mission);
});

// Endpoint to update a mission
app.MapPut("/missions/{id}", async (int id, Mission updatedMission, FullStackContext db) =>
{
    var mission = await db.Missions.FindAsync(id);
    if (mission is null) return Results.NotFound();

    mission.Name = updatedMission.Name;
    mission.Waypoints = updatedMission.Waypoints;
    mission.CreatedAt = updatedMission.CreatedAt;

    await db.SaveChangesAsync();
    return Results.NoContent();
});
// Endpoint to delete a mission
app.MapDelete("/missions/{id}", async (int id, FullStackContext db) =>
{
    if (await db.Missions.FindAsync(id) is Mission mission)
    {
        db.Missions.Remove(mission);
        await db.SaveChangesAsync();
        return Results.Ok(mission);
    }

    return Results.NotFound();
});

// Endpoint to get all paths
app.MapGet("/paths", async (FullStackContext db) =>
    await db.MapPaths.ToListAsync());

// Endpoint to get a single path by id
app.MapGet("/paths/{id}", async (int id, FullStackContext db) =>
    await db.MapPaths.FindAsync(id) is MapPath mapPath ? Results.Ok(mapPath) : Results.NotFound());

// Endpoint to create a new path
app.MapPost("/paths", async (MapPath mapPath, FullStackContext db) =>
{
    // Add the path to the database
    db.MapPaths.Add(mapPath);

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return the created path with the appropriate status code and location header
    return Results.Created($"/paths/{mapPath.Id}", mapPath);
});


// Endpoint to update a path
app.MapPut("/paths/{id}", async (int id, MapPath updatedPath, FullStackContext db) =>
{
    var mapPath = await db.MapPaths.FindAsync(id);
    if (mapPath is null) return Results.NotFound();

    mapPath.Name = updatedPath.Name;
    mapPath.Map = updatedPath.Map;
    mapPath.PosX = updatedPath.PosX;
    mapPath.PosY = updatedPath.PosY;
    mapPath.Orientation = updatedPath.Orientation;

    await db.SaveChangesAsync();
    return Results.NoContent();
});
// Endpoint to delete all paths
app.MapDelete("/paths/delete-all", async (FullStackContext db) =>
{
    // Ambil semua path dari database
    var allPaths = await db.MapPaths.ToListAsync();

    // Hapus setiap path dari database
    db.MapPaths.RemoveRange(allPaths);

    // Simpan perubahan ke database
    await db.SaveChangesAsync();

    // Mengembalikan respons dengan status OK
    return Results.Ok("All paths deleted successfully");
});

// Endpoint to delete a path
app.MapDelete("/paths/{id}", async (int id, FullStackContext db) =>
{
    if (await db.MapPaths.FindAsync(id) is MapPath mapPath)
    {
        db.MapPaths.Remove(mapPath);
        await db.SaveChangesAsync();
        return Results.Ok(mapPath);
    }

    return Results.NotFound();
});

// Endpoint to get all footprints
app.MapGet("/footprints", async (FullStackContext db) =>
    await db.Footprints.ToListAsync());



app.MapGet("/footprints/{id}", async (int id, FullStackContext db) =>
    await db.Footprints.FindAsync(id) is Footprint footprint ? Results.Ok(footprint) : Results.NotFound());

// Endpoint to create a new footprint
app.MapPost("/footprints", async (HttpContext context, FullStackContext db) =>
{
    try
    {
        var form = await context.Request.ReadFormAsync();

        var name = form["Name"];
        var robotname = form["Robotname"];

        // Process the image file
        var file = form.Files["file"];

        // Convert the image file to binary data
        byte[] imageData;
        using (var memoryStream = new MemoryStream())
        {
            await file.CopyToAsync(memoryStream);
            imageData = memoryStream.ToArray();
        }

        // Add the footprint to the database
        var newFootprint = new Footprint
        {
            Name = name,
            Robotname = robotname,
            ImageData = imageData
        };

        db.Footprints.Add(newFootprint);

        // Save changes to the database
        await db.SaveChangesAsync();

        // Return the created footprint with the appropriate status code and location header
        context.Response.Headers["Location"] = $"/footprints/{newFootprint.Id}";
        context.Response.StatusCode = 201; // Created

        // Optionally, you can return a response body with the created footprint
        await context.Response.WriteAsync(JsonSerializer.Serialize(newFootprint));
    }
    catch (Exception ex)
    {
        context.Response.StatusCode = 500; // Internal Server Error
        await context.Response.WriteAsync($"Error: {ex.Message}");
    }
});



// Endpoint to update a footprint
app.MapPut("/footprints/{id}", async (int id, Footprint updatedFootprint, FullStackContext db) =>
{
    var footprint = await db.Footprints.FindAsync(id);
    if (footprint is null) return Results.NotFound();

    footprint.Name = updatedFootprint.Name;
    footprint.Robotname = updatedFootprint.Robotname;
    footprint.ImageData = updatedFootprint.ImageData;

    await db.SaveChangesAsync();
    return Results.NoContent();
});

// Endpoint to delete a footprint
app.MapDelete("/footprints/{id}", async (int id, FullStackContext db) =>
{
    if (await db.Footprints.FindAsync(id) is Footprint footprint)
    {
        db.Footprints.Remove(footprint);
        await db.SaveChangesAsync();
        return Results.Ok(footprint);
    }

    return Results.NotFound();
});
// Endpoint to get all moduls
app.MapGet("/moduls", async (FullStackContext db) =>
    await db.Moduls.ToListAsync());

// Endpoint to get a single modul by id
app.MapGet("/moduls/{id}", async (int id, FullStackContext db) =>
    await db.Moduls.FindAsync(id) is Modul modul ? Results.Ok(modul) : Results.NotFound());

// Endpoint to create a new modul
app.MapPost("/moduls", async (Modul modul, FullStackContext db) =>
{
    // Add the modul to the database
    db.Moduls.Add(modul);

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return the created modul with the appropriate status code and location header
    return Results.Created($"/moduls/{modul.Id}", modul);
});

// Endpoint to update a modul
app.MapPut("/moduls/{id}", async (int id, Modul updatedModul, FullStackContext db) =>
{
    var modul = await db.Moduls.FindAsync(id);
    if (modul is null) return Results.NotFound();

    modul.Name = updatedModul.Name;
    modul.Date = updatedModul.Date;
    modul.PortIn = updatedModul.PortIn;
    modul.PortOut = updatedModul.PortOut;

    await db.SaveChangesAsync();
    return Results.NoContent();
});

// Endpoint to delete a modul
app.MapDelete("/moduls/{id}", async (int id, FullStackContext db) =>
{
    if (await db.Moduls.FindAsync(id) is Modul modul)
    {
        db.Moduls.Remove(modul);
        await db.SaveChangesAsync();
        return Results.Ok(modul);
    }

    return Results.NotFound();
});
// Endpoint to get all errors
app.MapGet("/errors", async (FullStackContext db) =>
    await db.Errors.ToListAsync());

// Endpoint to get a single error by id
app.MapGet("/errors/{id}", async (int id, FullStackContext db) =>
    await db.Errors.FindAsync(id) is Error error ? Results.Ok(error) : Results.NotFound());

app.MapPost("/errors", async (Error error, FullStackContext db) =>
{
    // Add the error to the database
    db.Errors.Add(error);

    // Mendapatkan waktu saat ini dalam zona waktu UTC
    DateTime utcNow = DateTime.UtcNow;

    // Mendapatkan zona waktu Jakarta
    TimeZoneInfo jakartaTimeZone = TimeZoneInfo.FindSystemTimeZoneById("SE Asia Standard Time"); // Waktu Indonesia Barat (WIB)

    // Konversi waktu UTC menjadi waktu Jakarta
    DateTime jakartaTime = TimeZoneInfo.ConvertTimeFromUtc(utcNow, jakartaTimeZone);

    // Create a new activity entry based on the error information
    var newActivity = new Activitie
    {
        Robotname = error.Robotname,
        Date = jakartaTime, // Menggunakan tanggal dan waktu sesuai dengan WIB
        Activity = $"Error occurred: {error.Explained}"
    };

    // Set expiry date 24 jam setelah waktu input
    newActivity.ExpiryTime = newActivity.Date.AddHours(24);

    // Tambahkan aktivitas ke database
    db.Activities.Add(newActivity);

    // Simpan perubahan ke database
    await db.SaveChangesAsync();

    // Return the created error with the appropriate status code and location header
    return Results.Created($"/errors/{error.Id}", error);
});


// Endpoint to update an error
app.MapPut("/errors/{id}", async (int id, Error updatedError, FullStackContext db) =>
{
    var error = await db.Errors.FindAsync(id);
    if (error is null) return Results.NotFound();

    error.Robotname = updatedError.Robotname;
    error.Date = updatedError.Date;
    error.Explained = updatedError.Explained;

    await db.SaveChangesAsync();
    return Results.NoContent();
});

// Endpoint to delete an error
app.MapDelete("/errors/{id}", async (int id, FullStackContext db) =>
{
    if (await db.Errors.FindAsync(id) is Error error)
    {
        db.Errors.Remove(error);
        await db.SaveChangesAsync();
        return Results.Ok(error);
    }

    return Results.NotFound();
});
// Endpoint to get activities by ID
app.MapGet("/activities/{id}", async (int id, FullStackContext db) =>
    await db.Activities.FindAsync(id) is Activitie activitie ? Results.Ok(activitie) : Results.NotFound());

    // Endpoint to get activities by time range and robot name
    // Endpoint untuk mendapatkan aktivitas berdasarkan rentang waktu dan nama robot
    app.MapGet("/activities/robot/{robotName}", async (string robotName, DateTimeOffset? startTime, DateTimeOffset? endTime, FullStackContext db) =>
    {
        IQueryable<Activitie> query = db.Activities.Where(a => a.Robotname == robotName);

        if (startTime.HasValue)
        {
            DateTime startTimeValue = startTime.Value.LocalDateTime; // Assign the local time value to a local variable
            query = query.Where(a => a.Date >= startTimeValue);
        }

        if (endTime.HasValue)
        {
            DateTime endTimeValue = endTime.Value.LocalDateTime; // Assign the local time value to a local variable
            query = query.Where(a => a.Date <= endTimeValue);
        }

        return Results.Ok(await query.ToListAsync());
    });


// Endpoint to create a new activity
app.MapPost("/activities", async (Activitie activitie, FullStackContext db) =>
{
    // Add the activity to the database
    db.Activities.Add(activitie);

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return the created activity with appropriate status code and location header
    return Results.Created($"/activities/{activitie.Id}", activitie);
});

// Endpoint to update an activity
app.MapPut("/activities/{id}", async (int id, Activitie updatedActivitie, FullStackContext db) =>
{
    var activitie = await db.Activities.FindAsync(id);
    if (activitie is null) return Results.NotFound();

    // Update activity properties with the updated ones
    activitie.Robotname = updatedActivitie.Robotname;
    activitie.Date = updatedActivitie.Date;
    activitie.Activity = updatedActivitie.Activity;

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return response with no content
    return Results.NoContent();
});

// Endpoint to delete an activity
app.MapDelete("/activities/{id}", async (int id, FullStackContext db) =>
{
    var activitie = await db.Activities.FindAsync(id);
    if (activitie is null) return Results.NotFound();

    // Remove the activity from the database
    db.Activities.Remove(activitie);

    // Save changes to the database
    await db.SaveChangesAsync();

    // Return response with the deleted activity
    return Results.Ok(activitie);
});



Process currentProcess = null;

// API untuk menghentikan proses peluncuran map
app.MapPost("/maps/stop", async (HttpContext context) =>
{
    try
    {
     if (currentProcess != null && !currentProcess.HasExited)
                        {
                            currentProcess.Kill();
                            Console.WriteLine("Map launch process terminated.");
                            return Results.Ok("Map launch process terminated.");
                        }
                        else
                        {
                            Console.WriteLine("No map launch process is currently running.");
                            return Results.BadRequest("No map launch process is currently running.");
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Error stopping map launch process: {ex.Message}");
                        return Results.Problem($"Error stopping map launch process: {ex.Message}");
                    }
                });


app.MapPost("/maps/launch", async (HttpContext context) =>
{
    var form = await context.Request.ReadFormAsync();
    var mapName = form["mapName"].ToString();
    Console.WriteLine($"Received map name: {mapName}");

    if (string.IsNullOrEmpty(mapName))
    {
        return Results.BadRequest("Map name is required.");
    }

    string tempDirectory = Path.Combine(Directory.GetCurrentDirectory(), "temp");
    string mapFilePath = Path.Combine(tempDirectory, mapName, $"{mapName}.yaml");

    if (!File.Exists(mapFilePath))
    {
        return Results.NotFound("Map file not found.");
    }

    // Stop previous map launch process if running
    if (currentProcess != null && !currentProcess.HasExited)
    {
        currentProcess.Kill();
        Console.WriteLine("Previous map launch process terminated.");
        currentProcess = null;
    }

    string command = $"ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:={mapFilePath}";

    var process = new Process()
    {
        StartInfo = new ProcessStartInfo
        {
            FileName = "/bin/bash",
            Arguments = $"-c \"{command}\"",
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true,
        }
    };

    currentProcess = process;

    // Event handler untuk output proses
    process.OutputDataReceived += (sender, e) =>
    {
        if (!string.IsNullOrEmpty(e.Data))
        {
            Console.WriteLine($"[ROS2 Output]: {e.Data}");
        }
    };

    // Event handler untuk error output proses
    process.ErrorDataReceived += (sender, e) =>
    {
        if (!string.IsNullOrEmpty(e.Data))
        {
            Console.WriteLine($"[ROS2 Error]: {e.Data}");
        }
    };

    process.Start();
    process.BeginOutputReadLine();
    process.BeginErrorReadLine();

    // Tunggu proses selesai berjalan
    await process.WaitForExitAsync();

    if (process.ExitCode != 0)
    {
        // Handle error using event handlers or other methods, not ReadToEndAsync
        return Results.Problem($"Failed to launch map. Check logs for details.");
    }

    Console.WriteLine("Map launched successfully.");
    return Results.Ok("Map launched successfully.");
});

app.MapGet("/maps", async (FullStackContext db) =>
{
    var maps = await db.Maps.ToListAsync();
    return Results.Ok(maps);
});

// Endpoint to get a single map by id
app.MapGet("/maps/{id}", async (int id, FullStackContext db) =>
    await db.Maps.FindAsync(id) is Map map ? Results.Ok(map) : Results.NotFound());

// Endpoint to create a new map
app.MapPost("/maps", async (Map map, FullStackContext db) =>
{
    map.Timestamp = DateTime.Now;

    db.Maps.Add(map);
    await db.SaveChangesAsync();

    return Results.Created($"/maps/{map.Id}", map);
});

// // Endpoint to save map files
app.MapPost("/maps/save", async (MapDto mapDto, FullStackContext db) =>
{
    string tempDirectory = "./temp"; // Use a directory in the project folder
    if (!Directory.Exists(tempDirectory))
    {
        Directory.CreateDirectory(tempDirectory);
    }

    // Sanitize the name to avoid any illegal characters in the file system
    string sanitizedMapName = string.Concat(mapDto.Name.Split(Path.GetInvalidFileNameChars()));
    string mapsDirectory = Path.Combine(tempDirectory, sanitizedMapName);
    if (!Directory.Exists(mapsDirectory))
    {
        try
        {
            Directory.CreateDirectory(mapsDirectory);
            Console.WriteLine($"Created directory: {mapsDirectory}");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Failed to create directory: {mapsDirectory}");
            Console.WriteLine($"Error: {ex.Message}\n{ex.StackTrace}");
            return Results.Problem("Failed to create directory for saving map files.");
        }
    }

    string pgmFilePath = Path.Combine(mapsDirectory, $"{sanitizedMapName}.pgm");
    string yamlFilePath = Path.Combine(mapsDirectory, $"{sanitizedMapName}.yaml");

    // Ensure the ROS2 command uses the correct path
    string command = $"ros2 run nav2_map_server map_saver_cli -f {Path.Combine(mapsDirectory, sanitizedMapName)}";

    try
    {
        var process = new Process()
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = "/bin/bash",
                Arguments = $"-c \"{command}\"",
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true,
            }
        };

        process.Start();
        string output = await process.StandardOutput.ReadToEndAsync();
        string error = await process.StandardError.ReadToEndAsync();
        process.WaitForExit();

        if (process.ExitCode != 0)
        {
            throw new Exception($"Command failed with exit code {process.ExitCode}: {error}");
        }

        Console.WriteLine($"Command Output: {output}");
        Console.WriteLine($"PGM File Path: {pgmFilePath}");
        Console.WriteLine($"YAML File Path: {yamlFilePath}");

        if (!File.Exists(pgmFilePath) || !File.Exists(yamlFilePath))
        {
            throw new Exception("Failed to save map files.");
        }

        // Read file contents
        byte[] pgmFileContent = await File.ReadAllBytesAsync(pgmFilePath);
        byte[] yamlFileContent = await File.ReadAllBytesAsync(yamlFilePath);

        var map = new Map
        {
            Name = mapDto.Name,
            Site = mapDto.Site,
            PgmFilePath = pgmFilePath,
            YamlFilePath = yamlFilePath,
            Timestamp = DateTime.Now
        };

        Console.WriteLine($"Map data read successfully.");

        db.Maps.Add(map);
        int saved = await db.SaveChangesAsync();
        if (saved <= 0)
        {
            throw new Exception("Failed to save map to the database. No data saved.");
        }

        Console.WriteLine($"Map data saved to database successfully.");

        return Results.Ok(map);
    }
    catch (Exception ex)
    {
        // Log the exception message and stack trace
        Console.WriteLine($"Error: {ex.Message}\n{ex.StackTrace}");
        return Results.Problem(ex.Message);
    }
});


// Endpoint to update a map
app.MapPut("/maps/{id}", async (int id, Map mapUpdate, FullStackContext db) =>
{
    var map = await db.Maps.FindAsync(id);

    if (map is null)
    {
        return Results.NotFound();
    }

    map.Name = mapUpdate.Name;
    map.Site = mapUpdate.Site;
    map.Timestamp = DateTime.Now;

    await db.SaveChangesAsync();

    return Results.Ok(map);
});

// Endpoint to delete a map
app.MapDelete("/maps/{id}", async (int id, FullStackContext db) =>
{
    var map = await db.Maps.FindAsync(id);

    if (map is null)
    {
        return Results.NotFound();
    }

    // Simpan path file sebelum dihapus
    string pgmFilePath = map.PgmFilePath;
    string yamlFilePath = map.YamlFilePath;

    // Hapus map dari database
    db.Maps.Remove(map);
    await db.SaveChangesAsync();

    // Hapus file dari direktori temp jika ada
    try
    {
        if (File.Exists(pgmFilePath))
        {
            File.Delete(pgmFilePath);
        }
        if (File.Exists(yamlFilePath))
        {
            File.Delete(yamlFilePath);
        }

        Console.WriteLine($"Deleted map files from temp directory: {pgmFilePath}, {yamlFilePath}");
    }
    catch (Exception ex)
    {
        // Jika gagal menghapus file, log pesan kesalahan
        Console.WriteLine($"Failed to delete map files from temp directory: {ex.Message}\n{ex.StackTrace}");
        // Anda bisa mengembalikan Results.Problem() dengan pesan kesalahan yang sesuai
        return Results.Problem("Failed to delete map files from temp directory.");
    }

    // Jika berhasil, kembalikan NoContent()
    return Results.NoContent();
});


app.MapGet("/start_mapping", async context =>
{
     try
                    {
                        string command = "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True";

                        if (currentProcess != null && !currentProcess.HasExited)
                        {
                            currentProcess.Kill();
                            Console.WriteLine("Previous mapping process terminated.");
                            currentProcess = null;
                        }

                        var process = new Process
                        {
                            StartInfo = new ProcessStartInfo
                            {
                                FileName = "/bin/bash",
                                Arguments = $"-c \"{command}\"",
                                RedirectStandardOutput = true,
                                RedirectStandardError = true,
                                UseShellExecute = false,
                                CreateNoWindow = true,
                            }
                        };

                        currentProcess = process;

                        process.OutputDataReceived += (sender, e) =>
                        {
                            if (!string.IsNullOrEmpty(e.Data))
                            {
                                Console.WriteLine($"[ROS2 Output]: {e.Data}");
                            }
                        };

                        process.ErrorDataReceived += (sender, e) =>
                        {
                            if (!string.IsNullOrEmpty(e.Data))
                            {
                                Console.WriteLine($"[ROS2 Error]: {e.Data}");
                            }
                        };

                        process.Start();
                        process.BeginOutputReadLine();
                        process.BeginErrorReadLine();

                        await process.WaitForExitAsync();

                        if (process.ExitCode != 0)
                        {
                            string error = await process.StandardError.ReadToEndAsync();
                            await context.Response.WriteAsJsonAsync(new { status = "error", message = $"Failed to start mapping: {error}" });
                        }
                        else
                        {
                            await context.Response.WriteAsJsonAsync(new { status = "success", message = "Mapping started successfully." });
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Error starting mapping process: {ex.Message}");
                        await context.Response.WriteAsJsonAsync(new { status = "error", message = ex.Message });
                    }
                });

app.MapGet("/stop_mapping", async context =>
{
   try
                    {
                        if (currentProcess != null && !currentProcess.HasExited)
                        {
                            currentProcess.Kill();
                            Console.WriteLine("Mapping process stopped.");
                            currentProcess = null;
                            await context.Response.WriteAsJsonAsync(new { status = "success", message = "Mapping stopped successfully." });
                        }
                        else
                        {
                            await context.Response.WriteAsJsonAsync(new { status = "error", message = "No mapping process running" });
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Error stopping mapping process: {ex.Message}");
                        await context.Response.WriteAsJsonAsync(new { status = "error", message = ex.Message });
                    }
                });

app.UseCors();
app.Run();


public class FullStackContext : DbContext
{
    public DbSet<User> Users { get; set; } //Table User
    public DbSet<Robot> Robots { get; set; } //Table Robot
    public DbSet<Map> Maps { get; set; } //Table Map
    public DbSet<MapPath> MapPaths { get; set; } //Table Path
    public DbSet<Mission> Missions { get; set; } //Table Mission
    public DbSet<Footprint> Footprints { get; set; } //Table Footprnt
    public DbSet<Modul> Moduls { get; set; } //Table Modul
    public DbSet<Error> Errors {get; set;}//table error
    public DbSet<Activitie> Activities {get; set;}//tabel activities
   
    public FullStackContext(DbContextOptions<FullStackContext> options) : base(options) { }

}

//Table Name
public class User
{
    public int Id { get; set; }
    public string? Username { get; set; }
    public string? Password { get; set; }
    public string? Phone { get; set; }
}


public class Robot
{
    public int Id { get; set; }
    public string? Name { get; set; }
    public string? Port { get; set; }
    public string? Ip { get; set; }
}
public class Map
{
    public int Id { get; set; }
    public string Name { get; set; }
    public string Site { get; set; }
    public string PgmFilePath { get; set; }
    public string YamlFilePath { get; set; }
    public DateTime Timestamp { get; set; }
}

public class MapDto
{
    public string Name { get; set; }
    public string Site { get; set; }
}

public class MapPath
{
    public int Id { get; set; }
    public string? Name { get; set; }
    public string? Map { get; set; }
    public double? PosX { get; set; }
    public double? PosY { get; set; }
    public double? Orientation { get; set; }
}

public class Mission
{
    public int Id { get; set; }
    public string Name { get; set; }
    public List<Waypoint> Waypoints { get; set; } = new();
    public DateTime CreatedAt { get; set; }
}

public class Waypoint
{
    public int Id { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Orientation { get; set; }
}

public class Footprint
{
    public int Id { get; set; }
    public string? Name { get; set; }
    public string? Robotname { get; set; }
    public byte[]? ImageData { get; set; } // This column will store the binary image data
}

public class Modul
{
    public int Id { get; set; }
    public string? Name { get; set; }
    public string? Date { get; set; }
    public string? PortIn { get; set; } 
    public string? PortOut { get; set; }// This column will store the binary image data
}
public class Error
{
    public int Id { get; set; }
    public string? Robotname { get; set; }
    public string? Date { get; set; }
    public string? Explained { get; set; } 
   
}
public class Activitie
{
    public int Id { get; set; }
    public string? Robotname { get; set; }
    public DateTime Date { get; set; } // Menggunakan DateTime untuk tanggal dan waktu
    public string? Activity { get; set; } // Mengubah Activitiy menjadi Activity
    public DateTime ExpiryTime { get; set; } // Add ExpiryTime property
}


