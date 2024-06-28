using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class path : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.RenameColumn(
                name: "Start",
                table: "MapPaths",
                newName: "PosY");

            migrationBuilder.RenameColumn(
                name: "Goal",
                table: "MapPaths",
                newName: "PosX");

            migrationBuilder.RenameColumn(
                name: "Distance",
                table: "MapPaths",
                newName: "Orientation");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.RenameColumn(
                name: "PosY",
                table: "MapPaths",
                newName: "Start");

            migrationBuilder.RenameColumn(
                name: "PosX",
                table: "MapPaths",
                newName: "Goal");

            migrationBuilder.RenameColumn(
                name: "Orientation",
                table: "MapPaths",
                newName: "Distance");
        }
    }
}
