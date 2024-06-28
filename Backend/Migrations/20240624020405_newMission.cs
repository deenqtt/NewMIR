using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class newMission : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "GoalOrientation",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "GoalX",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "GoalY",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "StartOrientation",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "StartX",
                table: "Missions");

            migrationBuilder.DropColumn(
                name: "StartY",
                table: "Missions");

            migrationBuilder.CreateTable(
                name: "Waypoint",
                columns: table => new
                {
                    Id = table.Column<int>(type: "INTEGER", nullable: false)
                        .Annotation("Sqlite:Autoincrement", true),
                    X = table.Column<double>(type: "REAL", nullable: false),
                    Y = table.Column<double>(type: "REAL", nullable: false),
                    Orientation = table.Column<double>(type: "REAL", nullable: false),
                    MissionId = table.Column<int>(type: "INTEGER", nullable: true)
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_Waypoint", x => x.Id);
                    table.ForeignKey(
                        name: "FK_Waypoint_Missions_MissionId",
                        column: x => x.MissionId,
                        principalTable: "Missions",
                        principalColumn: "Id");
                });

            migrationBuilder.CreateIndex(
                name: "IX_Waypoint_MissionId",
                table: "Waypoint",
                column: "MissionId");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropTable(
                name: "Waypoint");

            migrationBuilder.AddColumn<double>(
                name: "GoalOrientation",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "GoalX",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "GoalY",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "StartOrientation",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "StartX",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);

            migrationBuilder.AddColumn<double>(
                name: "StartY",
                table: "Missions",
                type: "REAL",
                nullable: false,
                defaultValue: 0.0);
        }
    }
}
